package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.TrajectorySequence;
import com.reefsharklibrary.pathing.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Disabled
@Autonomous
public class RedLeft extends LinearOpMode {
    private enum AutoState{
        PLACING_PRELOAD,
        PLACING_DELAY,
        CYCLE_1,
        CYCLE_1_WAIT,
        PLACING_1,
        CYCLE_2,
        CYCLE_2_WAIT,
        PLACING_2,
        CYCLE_3,
        CYCLE_3_WAIT,
        PLACING_3,
        PARK,
        FINISHED
    }

    RedLeft.AutoState autoState = RedLeft.AutoState.PLACING_PRELOAD;

    ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    OldLocalizer oldLocalizer;
    Intake intake;
    Outtake outtake;
    //    TeleopController teleopController;
    Transfer transfer;
    MasterThread masterThread;
    Telemetry.Item loopTime;
    Telemetry.Item temporalCount;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());
        temporalCount = telemetry.addData("Temporal data", "");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        oldLocalizer = new OldLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), oldLocalizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.FOLLOW_PATH);

        intake = new Intake(masterThread.getData());
        transfer = new Transfer(masterThread.getData());
        outtake = new Outtake(masterThread.getData(), true, false);

//        teleopController = new TeleopController(intake, transfer, outtake, masterThread.getData());


        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer,
                intake,
                transfer,
                outtake
        );


        TrajectorySequence preload = new TrajectorySequenceBuilder(new Pose2d(16.8, -62.1, Math.toRadians(90)), RobotConstants.constraints)
                .splineToConstantHeading(new Vector2d(10, -32.5), Math.toRadians(90))
                .callMarker(2, () -> {
                    outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_FRONT);
                })

                .callMarkerFromEnd(.2, () -> {
                    outtake.place();
                })
                .setEndDelay(1.5)
                .build();

        TrajectorySequence cycle1 = new TrajectorySequenceBuilder(preload.endPose(), RobotConstants.constraints)
                .back(2)
                .callMarker(.5, () -> {
                    outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
                })
                .splineToConstantHeading(new Vector2d(49, -42.5), Math.toRadians(180))
                .callMarker(24, () -> {
                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                })
                .callMarkerFromEnd(.5, () -> {
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(1)
                .back(.1)
                .localTemporalMarker(0, () -> {
                    intake.retract();
                })
                .splineToSplineHeading(new Pose2d(55, -55, Math.toRadians(45)), Math.toRadians(225))
                .setTargetEndDistance(.5)
                .build();

        TrajectorySequence cycle2 = new TrajectorySequenceBuilder(cycle1.endPose(), RobotConstants.constraints)
                .forward(2)
                .callMarker(.5, () -> {
                    outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
                })
                .splineToSplineHeading(new Pose2d(51, -44.3, Math.toRadians(90)), Math.toRadians(90))
                .callMarker(5, () -> {
                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                })
                .forward(1)
                .left(8)
                .localCallMarkerFromEnd(.5, () -> {
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(1)
                .right(.4)
                .localTemporalMarker(0, () -> {
                    intake.retract();
                })
                .splineToLineHeading(new Pose2d(55, -55, Math.toRadians(45)), Math.toRadians(225))
                .setTargetEndDistance(.5)
                .build();

//        TrajectorySequence cycle3 = new TrajectorySequenceBuilder(cycle2.endPose(), RobotConstants.constraints)
//                .forward(2)
//                .callMarker(.5, () -> {
//                    outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
//                })
//                .splineToSplineHeading(new Pose2d(52, 42, Math.toRadians(300)), Math.toRadians(300))
//                .callMarker(5, () -> {
//                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
//                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
//                })
//                .left(8)
//                .localCallMarkerFromEnd(1, () -> {
//                    intake.setTargetIntakeSpeed(1);
//                })
//                .setEndDelay(1)
//                .back(.1)
//                .localTemporalMarker(0, () -> {
//                    intake.retract();
//                })
//                .splineToSplineHeading(new Pose2d(55, 55, Math.toRadians(270-45)), Math.toRadians(45))
//                .setTargetEndDistance(.5)
//                .build();

        TrajectorySequence park = new TrajectorySequenceBuilder(cycle2.endPose(), RobotConstants.constraints)
                .splineToSplineHeading(new Pose2d(26,-13, Math.toRadians(180)), Math.toRadians(0))
                .callMarker(7, () -> {
                    outtake.setTargetSlidePos(Outtake.VerticalSlide.DOWN);
                })
                .back(17)
                .build();



        drivetrain.setForwardComponent(.5);

        waitForStart();
        drivetrain.followTrajectorySequence(preload);

        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());




        while ( !isStopRequested()) {

            switch (autoState) {
                case PLACING_PRELOAD:
                    if (drivetrain.isFinished()) {
//                        autoTimer.reset();
                        drivetrain.followTrajectorySequence(cycle1);

                        autoState = RedLeft.AutoState.CYCLE_1;
                    }
                    break;
                case CYCLE_1:
                    if (drivetrain.isFinished() && Math.abs(outtake.getSlideError())<.5) {
                        autoState = RedLeft.AutoState.CYCLE_1_WAIT;
                        autoTimer.reset();
                    }
                    break;
                case CYCLE_1_WAIT:
                    if (autoTimer.seconds()>.5) {
                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoState = RedLeft.AutoState.PLACING_1;
                        autoTimer.reset();
                    }
                    break;
                case PLACING_1:
                    if (autoTimer.seconds()>.4) {
                        drivetrain.followTrajectorySequence(cycle2);
                        autoState = RedLeft.AutoState.CYCLE_2;

                    }
                    break;
                case CYCLE_2:
                    if (drivetrain.isFinished() && Math.abs(outtake.getSlideError())<.5) {
                        autoState = RedLeft.AutoState.CYCLE_2_WAIT;
                        autoTimer.reset();
                    }
                    break;
                case CYCLE_2_WAIT:
                    if (autoTimer.seconds()>.5) {
                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoState = RedLeft.AutoState.PLACING_2;
                        autoTimer.reset();
                    }
                    break;
                case PLACING_2:
                    if (autoTimer.seconds()>.4) {
                        drivetrain.followTrajectorySequence(park);
                        autoState = RedLeft.AutoState.PARK;

                    }
                    break;
//                case CYCLE_3:
//                    if (drivetrain.isFinished() && Math.abs(outtake.getSlideError())<.5) {
//                        autoState = AutoState.CYCLE_3_WAIT;
//                        autoTimer.reset();
//                    }
//                    break;
//                case CYCLE_3_WAIT:
//                    if (autoTimer.seconds()>.2) {
//                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);
//
//                        autoState = AutoState.PLACING_3;
//                        autoTimer.reset();
//                    }
//                    break;
//                case PLACING_3:
//                    if (autoTimer.seconds()>.5) {
//                        drivetrain.followTrajectorySequence(park);
//                        autoState = AutoState.PARK;
//
//                    }
//                    break;
                case PARK:
                    if (drivetrain.isFinished()) {
                        autoState = RedLeft.AutoState.FINISHED;
                    }
                    break;

            }

            masterThread.unThreadedUpdate();

            if (intake.transfered()) {
                outtake.grabFromTransfer();
            }

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
