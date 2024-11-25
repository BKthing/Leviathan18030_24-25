package org.firstinspires.ftc.teamcode.depricated;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.headingPID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.lateralPID;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.naturalDecel;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.trackWidth;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.localizers.OldLocalizer;
import com.reefsharklibrary.pathing.EndpointEstimator;
import com.reefsharklibrary.pathing.TrajectoryInterface;
import com.reefsharklibrary.pathing.TrajectorySequenceRunner;
import com.reefsharklibrary.pathing.data.IndexCallMarker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.Arrays;
import java.util.List;

public class LimitedDrivetrain extends SubSystem {

    public enum DriveState {
        FOLLOW_PATH,
        DRIVER_CONTROL
    }

    DriveState driveState;

    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    List<Double> lastPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);

    Telemetry.Item motorPowerTelemetry;

    Telemetry.Item followState;

    Telemetry.Item targetMotionState;

    //how much a motor power must change to warrant an update
    private final double minPowerChange = .03;


    VoltageSensor batteryVoltageSensor;
    double voltage = 0;

    OldLocalizer localizer;

    TrajectorySequenceRunner runner;

    Pose2d poseEstimate;
    Pose2d poseVelocity;
    Pose2d poseAcceleration;

    boolean fieldCentric = false;
    double headingOffset = 0;

    public LimitedDrivetrain(SubSystemData data, OldLocalizer localizer) {
        this(data, localizer, DriveState.FOLLOW_PATH);
    }

    public LimitedDrivetrain(SubSystemData data, OldLocalizer localizer, DriveState driveState) {
        super(data);

        this.localizer = localizer;

        this.driveState = driveState;

        runner = new TrajectorySequenceRunner(lateralPID, headingPID, naturalDecel, trackWidth, RobotConstants.constraints);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");//ex 0
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");//
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");//ex 1

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontLeft.setDirection(DcMotorSimple.Dir
//        ection.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        motorPowerTelemetry = telemetry.addData("Motor powers", "");

        followState = telemetry.addData("Follow state", "");

        targetMotionState = telemetry.addData("Motion state", "");

    }

    @Override
    public void priorityData() {
        voltage = batteryVoltageSensor.getVoltage();
        switch (driveState) {
            case FOLLOW_PATH:
                //getting the data before the localizer starts updating it
                poseEstimate = localizer.getPoseEstimate();
                poseVelocity = localizer.getPoseVelocity();
                poseAcceleration = localizer.getPoseAcceleration();

                if (!poseVelocity.isFinite()) {
                    throw new RuntimeException("Invalid velocity:" + poseVelocity);
                }
                break;
            case DRIVER_CONTROL:
                //getting the data before the localizer starts updating it
                poseEstimate = localizer.getPoseEstimate();
                poseVelocity = localizer.getPoseVelocity();
                poseAcceleration = localizer.getPoseAcceleration();

                if (!poseVelocity.isFinite()) {
                    throw new RuntimeException("Invalid velocity:" + poseVelocity);
                }
                break;
        }
    }

    @Override
    public void loop() {
        switch (driveState) {
            case FOLLOW_PATH:
                setDrivePower(runner.update(poseEstimate, poseVelocity, poseAcceleration));

                break;
            case DRIVER_CONTROL:
                double relativeHeading = poseEstimate.getHeading()-headingOffset;

//                double speedMultiplier = 1- gamepad1.right_trigger*.7;
                double speedMultiplier = .45- gamepad1.right_trigger*.2;

                MotorPowers powers = new MotorPowers();

                double forward;
                if (Math.abs(gamepad1.left_stick_y)>.02) {
                    forward = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y)*speedMultiplier;
                } else {
                    forward = 0;
                }

                double strafing;
                if (Math.abs(gamepad1.left_stick_x)>.02) {
                    strafing = -gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x)*speedMultiplier;
                } else {
                    strafing = 0;
                }

                double turn;
                if (Math.abs(gamepad1.right_stick_x)>.02) {
                    turn = -gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*.8*speedMultiplier; //(gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x))*.7;
                } else {
                    turn = 0;
                }

                powers.addHeading(turn);

                if (fieldCentric) {
                    //converting to field centric
                    powers.addVector(new Vector2d(forward, strafing).rotate(-relativeHeading));

                } else {
                    powers.addVector(new Vector2d(forward, strafing));
                }


                //driving settings
//                if (gamepad1.back) {
//                    headingOffset = poseEstimate.getHeading();
//                }

//                if (gamepad1.x) {
//                    fieldCentric = false;
//                } else if (gamepad1.b) {
//                    fieldCentric = true;
//                }


                setDrivePower(powers);
                break;
        }
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        packet.fieldOverlay().setStrokeWidth(1);

        followState.setValue(runner.getFollowState());

        if (driveState == DriveState.DRIVER_CONTROL) {
            EndpointEstimator endpointController = new EndpointEstimator(lateralPID, headingPID, naturalDecel);

            endpointController.updateEndPos(poseEstimate, poseVelocity);

            DashboardUtil.drawRobot(packet.fieldOverlay(), endpointController.getEstimatedEndPos());
        }

        if (runner.getFollowState() != TrajectorySequenceRunner.FollowState.NO_TRAJECTORY) {

            for (TrajectoryInterface trajectory : runner.getTrajectorySequence().getTrajectories()) {
                List<Pose2d> poseList = trajectory.poseList();

                packet.fieldOverlay().setStroke("#FC0703");

                //drawing path segment
                DashboardUtil.drawSampledPath(packet.fieldOverlay(), poseList);

                //highlighting segment if it is part of the trajectory were following
                if (trajectory == runner.getTrajectorySequence().getCurrentTrajectory()) {
                    packet.fieldOverlay().setAlpha(.3);
                    packet.fieldOverlay().setStrokeWidth(3);

                    DashboardUtil.drawSampledPath(packet.fieldOverlay(), poseList);

                    packet.fieldOverlay().setAlpha(1);
                    packet.fieldOverlay().setStrokeWidth(1);
                }

                List<IndexCallMarker> callMarkerList = trajectory.callMarkerList();
                int callMarkerIndex = trajectory.callMarkerIndex();

                packet.fieldOverlay().setStroke("#4CAF50");

                //drawing markers that have been called
                for (int i = 0; i < callMarkerIndex; i++) {
                    DashboardUtil.drawMarker(packet.fieldOverlay(), poseList.get(callMarkerList.get(i).getCallPosition()).getVector2d(), false);
                }

                //drawing markers that haven't been called yet
                for (int i = callMarkerIndex; i < callMarkerList.size(); i++) {
                    DashboardUtil.drawMarker(packet.fieldOverlay(), poseList.get(callMarkerList.get(i).getCallPosition()).getVector2d(), true);
                }
            }

            if (runner.getFollowState() == TrajectorySequenceRunner.FollowState.FOLLOW_TRAJECTORY) {
                packet.fieldOverlay().setStroke("#4CAF50");
                Pose2d targetPose = runner.getTrajectorySequence().getCurrentTrajectory().getTargetPose();

                //drawing robot's target position
                DashboardUtil.drawRobot(packet.fieldOverlay(), targetPose);

                //drawing the robots target velocity
                targetMotionState.setValue(runner.getTrajectorySequence().getCurrentTrajectory().getTargetDirectionalPose().getDirection());
                DashboardUtil.drawArrow(packet.fieldOverlay(), targetPose.getVector2d(), targetPose.getVector2d().plus(runner.getTrajectorySequence().getCurrentTrajectory().getTargetDirectionalPose().getVector2d()));
            } else if (runner.getFollowState() == TrajectorySequenceRunner.FollowState.TARGET_END_POINT) {
                packet.fieldOverlay().setStroke("#4CAF50");

                //drawing where the robot predicts it will stop at
                DashboardUtil.drawRobot(packet.fieldOverlay(), runner.getEndpointController().getEstimatedEndPos());
            }

            //drawing robots motor powers as a vector
            Pose2d driveTrainMovement = MotorPowers.powersToPose(lastPowers);
            motorPowerTelemetry.setValue(driveTrainMovement.toStringRadians());

            packet.fieldOverlay().setStroke("#0a0a0f");

            DashboardUtil.drawArrow(packet.fieldOverlay(), localizer.getPoseEstimate().getVector2d(), localizer.getPoseEstimate().getVector2d().plus(driveTrainMovement.getVector2d().rotate(-localizer.getPoseEstimate().getHeading()).multiply(10)));
        }

        return packet;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public TrajectorySequenceRunner runner() {
        return runner;
    }

    public void setDrivePower(MotorPowers motorPowers) {
        List<Double> powers = motorPowers.getNormalizedVoltages(voltage);

        if (different(powers)) {
            hardwareQueue.add(() -> frontLeft.setPower(powers.get(0)));
            hardwareQueue.add(() -> frontRight.setPower(powers.get(3)));
            hardwareQueue.add(() -> backLeft.setPower(powers.get(1)));
            hardwareQueue.add(() -> backRight.setPower(powers.get(2)));

            lastPowers = powers;
        }
    }

    private boolean different(List<Double> powers) {
        for (int i = 0; i<4; i++) {
            //checks if powers are far enough apart to be worthy of an update
            if ((lastPowers.get(i) == 0 && powers.get(i) != 0) || (lastPowers.get(i) != 0 && powers.get(i) == 0) || (Math.abs(powers.get(i)-lastPowers.get(i))>.03)) {
                return true;
            }
        }

        return false;
    }
}
