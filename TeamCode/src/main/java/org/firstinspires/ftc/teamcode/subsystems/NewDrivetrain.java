package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.localizers.CluelessTwoWheelLocalizer;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive2;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.UnModifiedMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.Arrays;
import java.util.List;

public class NewDrivetrain extends SubSystem {

    public enum DriveState {
        FOLLOW_PATH,
        DRIVER_CONTROL
    }

    private DriveState driveState;

    public final MecanumDrive2 drive;

    private Action path;

    private boolean followPath = false;

//    private FtcDashboard dashboard;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private List<DcMotorEx> drivetrainMotors;

    private final List<ReusableHardwareAction> motorActions;

    private List<Double> actualPowers = Arrays.asList(0.0, 0.0, 0.0, 0.0);

    private final Telemetry.Item motorPowerTelemetry;

    private final Telemetry.Item followState;

    private final Telemetry.Item targetMotionState;

    private final Telemetry.Item forwardComponentTelemetry;

    private final Telemetry.Item radiansPerInch;

    private TelemetryPacket packet = new TelemetryPacket();

    private Canvas canvas = new Canvas();


    //how much a motor power must change to warrant an update
    private final double minPowerChange = .03;


    private final ReusableHardwareAction voltageSensorHardwareAction;
    private final VoltageSensor batteryVoltageSensor;
    private double voltage = 13, updatedVoltage = 13;

//    private final TrajectorySequenceRunner runner;

    private Pose2d roadRunnerPoseEstimate;
    private Pose2d roadRunnerPoseVelocity;
//    private Pose2d poseAcceleration;

    private boolean fieldCentric = false;
    private double headingOffset = 0;

    private final Telemetry.Item roadRunnerPos;
    private final Telemetry.Item roadRunnerVel;
    private final Telemetry.Item driveTrainLoopTime;

    private final ElapsedTimer driveTrainLoopTimer = new ElapsedTimer();

    private final ElapsedTimer voltageUpdateTimer = new ElapsedTimer();

    private final LazyImu lazyImu;

//    public final Localizer roadRunnerLocalizer;

    public final GoBildaPinpointDriverRR pinpointLocalizer;

    public com.acmerobotics.roadrunner.Pose2d roadRunnerPose = new com.acmerobotics.roadrunner.Pose2d(0, 0, 0);


    public NewDrivetrain(SubSystemData data, DcMotorEx parallelEncoder, DcMotorEx perpendicularEncoder) {
        this(data, parallelEncoder, perpendicularEncoder, DriveState.FOLLOW_PATH);
    }

    public NewDrivetrain(SubSystemData data, DcMotorEx parallelEncoder, DcMotorEx perpendicularEncoder, DriveState driveState) {
        super(data);

//        this.localizer = localizer;

        this.driveState = driveState;

        this.drive = new MecanumDrive2(hardwareMap, parallelEncoder, perpendicularEncoder, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        this.motorActions = Arrays.asList(new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue), new ReusableHardwareAction(hardwareQueue));

        this.voltageSensorHardwareAction = new ReusableHardwareAction(hardwareQueue);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");//ex 0
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");//
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");//ex 1

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrainMotors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        lazyImu = new LazyImu(hardwareMap, "expansionImu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

//        roadRunnerLocalizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), parallelEncoder, perpendicularEncoder, .94*2*Math.PI/2000);


        pinpointLocalizer = hardwareMap.get(GoBildaPinpointDriverRR.class,new PinpointDrive.Params().pinpointDeviceName);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        roadRunnerPos = telemetry.addData("Roadrunner pos", "");

        roadRunnerVel = telemetry.addData("Roadrunner vel", "");

        motorPowerTelemetry = telemetry.addData("Motor powers", "");

        followState = telemetry.addData("Follow state", "");

        targetMotionState = telemetry.addData("Motion state", "");

        forwardComponentTelemetry = telemetry.addData("Forward component", "");

        radiansPerInch = telemetry.addData("rad per inch", "");

        driveTrainLoopTime = telemetry.addData("Drivetrain loop time", "");
    }

    @Override
    public void priorityData() {
        voltage = updatedVoltage;

    }

    @Override
    public void loop() {

        driveTrainLoopTimer.reset();
        drive.setVoltage(voltage);

        pinpointLocalizer.update();

//        Twist2dDual<Time> twist = ;

        roadRunnerPose = pinpointLocalizer.getPositionRR();//MathUtil.toRoadRunnerPose(poseEstimate);//

        roadRunnerPoseEstimate = new Pose2d(roadRunnerPose.position.x, roadRunnerPose.position.y, roadRunnerPose.heading.toDouble());

        PoseVelocity2d velocity = pinpointLocalizer.getVelocityRR();

        roadRunnerPoseVelocity = new Pose2d(velocity.linearVel.x, velocity.linearVel.y, velocity.angVel);

        roadRunnerPos.setValue(roadRunnerPoseEstimate);
        roadRunnerVel.setValue(roadRunnerPoseVelocity);

//        drive.updatePoseEstimate();

        drive.updatePoseEstimate(roadRunnerPose, velocity);

        if (voltageUpdateTimer.milliSeconds()>200) {
            voltageSensorHardwareAction.setAndQueueIfEmpty(() -> {
                updatedVoltage = batteryVoltageSensor.getVoltage();
            });
            voltageUpdateTimer.reset();
        }


        switch (driveState) {
            case FOLLOW_PATH:

                if (followPath) {
                    packet = new TelemetryPacket();
                    packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

                    followPath = path.run(packet);


                    setDrivePower(drive.getDrivePowers());
                    followState.setValue("FOLLOW");
                }

                break;
            case DRIVER_CONTROL:
                followState.setValue("DRIVER");
                double relativeHeading = roadRunnerPoseEstimate.getHeading()-headingOffset;

                double speedMultiplier = 1- gamepad1.right_trigger*.7;

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
                    turn = -gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x*.4*speedMultiplier; //(gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x))*.7;
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
                if (gamepad1.back) {
                    headingOffset = roadRunnerPoseEstimate.getHeading();
                }

                if (gamepad1.start && gamepad1.dpad_up) {
                    fieldCentric = false;
                } else if (gamepad1.start && gamepad1.dpad_down) {
                    fieldCentric = true;
                }


                setDrivePower(powers);

                break;
        }
        motorPowerTelemetry.setValue(drive.getDrivePowers());
        driveTrainLoopTime.setValue(driveTrainLoopTimer.milliSeconds());


    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {

        if (driveState == DriveState.FOLLOW_PATH) {
            packet.fieldOverlay().getOperations().addAll(this.packet.fieldOverlay().getOperations());
        }

        return packet;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void followPath(Action path) {
        this.path = path;
        canvas = new Canvas();
        this.path.preview(canvas);
        followPath = true;
    }

    public boolean isFinished() {
        return !followPath;
    }

    public void setDrivePower(MotorPowers motorPowers) {
        setDrivePower(motorPowers.getNormalizedVoltages(voltage));
    }

    public void setDrivePower(List<Double> targetPowers) {
        for (int i = 0; i<4; i++) {
            if ((actualPowers.get(i) == 0 && targetPowers.get(i) != 0) || (actualPowers.get(i) != 0 && targetPowers.get(i) == 0) || (Math.abs(targetPowers.get(i)- actualPowers.get(i))>.1)) {
                int finalI = i;
                motorActions.get(i).setAndQueueAction(() -> {
                    drivetrainMotors.get(finalI).setPower(targetPowers.get(finalI));
                    actualPowers.set(finalI, targetPowers.get(finalI));
                });
            }
        }
    }

}
