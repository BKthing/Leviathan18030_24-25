package org.firstinspires.ftc.teamcode.depricated;

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
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Disabled
@Autonomous
public class LeftAuto extends LinearOpMode {

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
        PARK1,
        PARK2,
        FINISHED
    }

    AutoState autoState = AutoState.PLACING_PRELOAD;

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

        // ---------------------- PLACING THE PRELOADED SPECIMEN ON THE RUNGS ----------------------

        /* sets the robot to go to its starting position on the wall and face away from the wall */
        TrajectorySequence preload = new TrajectorySequenceBuilder(new Pose2d(16.8, 62.1, Math.toRadians(270)), RobotConstants.constraints)
                /* moves the robot to the rungs */
                .splineToConstantHeading(new Vector2d(10, 33), Math.toRadians(270))
                .callMarker(2, () -> {

                    /* rotates the outtake 4bar to a position where it can place the preloaded
                    specimen onto the top rung, and rotates the claw to hold the specimen right side up  */
                    outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_FRONT);
                })

                .callMarkerFromEnd(.2, () -> {
                    /* rotates the 4bar down to place the already-positioned specimen on the rungs */
                    outtake.place();
                })
                .setEndDelay(1.5)
                .build();

        // ---------------------- CYCLING THE FIRST SAMPLE (FROM SPIKE MARK) -----------------------

        /* sets the robot to go to the expected end position of the preload stage */
        TrajectorySequence cycle1 = new TrajectorySequenceBuilder(preload.endPose(), RobotConstants.constraints)
                .back(2)
                .callMarker(.5, () -> {
                    /* moves the outtake into a position where it is ready to grab the sample from
                    the transfer */
                    outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
                })
                /* moves the robot to next to the sample */
                .splineToConstantHeading(new Vector2d(49, 41), Math.toRadians(0))
                .callMarker(24, () -> {
                    drivetrain.setForwardComponent(.25);
                    /* extends the intake slides a preset amount and lowers the intake spinners */
                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                    /* sets the transfer ramp's rotation to horizontal and slightly extends the servos
                    arms to not interfere with the transfer process */
                    transfer.setTransferState(Transfer.TransferState.NEUTRAL);
                })
                .callMarkerFromEnd(.5, () -> {
//                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2 +2);
                    /* spins the intake spinners at a speed of 1 to pick up sample */
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(1)
                .back(.1)
                .localTemporalMarker(0, () -> {
                    /* retracts the intake back into the robot and places the sample into the transfer */
                    intake.retract();
                    drivetrain.setForwardComponent(.5);
                })
                /* rotates the robot towards the buckets and moves towards them */
                .splineToSplineHeading(new Pose2d(56, 56, Math.toRadians(270-45)), Math.toRadians(45))
                .setTargetEndDistance(.5)
                .build();

        // ---------------------- CYCLING THE SECOND SAMPLE (FROM SPIKE MARK) ----------------------

        /* sets the robot to go to the expected end position of the cycle1 stage */
        TrajectorySequence cycle2 = new TrajectorySequenceBuilder(cycle1.endPose(), RobotConstants.constraints)
                .forward(2)
                .callMarker(.5, () -> {
                    /* moves the outtake into a position where it is ready to grab the sample from
                    the transfer */
                    outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
                })
                /* moves the robot to next to the 2nd sample */
                .splineToSplineHeading(new Pose2d(50, 42.8, Math.toRadians(270)), Math.toRadians(270))
                .callMarker(3, () -> {
                    drivetrain.setForwardComponent(.25);
                    /* extends the intake slides a preset amount and lowers the intake spinners */
                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                    /* sets the transfer ramp's rotation to horizontal and slightly extends the servos
                    arms to not interfere with the transfer process */
                    transfer.setTransferState(Transfer.TransferState.NEUTRAL);
                })
                .forward(1)
                .left(8)
                .localCallMarkerFromEnd(.5, () -> {
                    /* spins the intake spinners at a speed of 1 to pick up sample */
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(1)
                .right(.4)
                .localTemporalMarker(0, () -> {
                    /* retracts the intake back into the robot and places the sample into the transfer */
                    intake.retract();
                    drivetrain.setForwardComponent(.5);
                })
                /* rotates the robot towards the buckets and moves towards them */
                .splineToLineHeading(new Pose2d(56, 56, Math.toRadians(270-45)), Math.toRadians(45))
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

        // ----------------------------------- PARKING THE ROBOT -----------------------------------

        /* sets the robot to go to the expected end position of the cycle2 stage */
        TrajectorySequence park1 = new TrajectorySequenceBuilder(cycle2.endPose(), RobotConstants.constraints)
                /* moves the robot to the parking area and changes the heading so that the outtake
                faces the hang bars */
                .splineToLineHeading(new Pose2d(20,13, Math.toRadians(0)), Math.toRadians(180))
                .callMarker(7, () -> {
                    /* lowers the outtake 4bar to touch the hang bars*/
                    outtake.setTargetSlidePos(Outtake.VerticalSlide.DOWN);
                })
                .build();

//        TrajectorySequence park2 = new TrajectorySequenceBuilder(park1.endPose(), RobotConstants.constraints)
//
//                .back(30)
//                .build();


        drivetrain.setForwardComponent(.5);

        /* makes the robot start the preload sequence when the auto is started */
        waitForStart();
        drivetrain.followTrajectorySequence(preload);

        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());


        //--------------- DOING EVERYTHING IN THE AUTO THAT ISN'T THE PRELOAD PROCESS --------------
        while ( !isStopRequested()) {

            switch (autoState) {
                /* does this code if the preload process is happening at the moment */
                case PLACING_PRELOAD:
                    /* starts cycle1 if the preload process is done */
                    if (drivetrain.isFinished()) {
//                        autoTimer.reset();
                        drivetrain.followTrajectorySequence(cycle1);

                        autoState = AutoState.CYCLE_1;
                    }
                    break;
                /* does this code if the cycle1 process is happening at the moment */
                case CYCLE_1:
                    /* switches the auto state to cycle1 WAIT if the cycle1 process is done */
                    if (drivetrain.isFinished() && Math.abs(outtake.getSlideError())<.5) {
                        autoState = AutoState.CYCLE_1_WAIT;
                        autoTimer.reset();
                    }
                    break;
                /* does this code if the cycle1 just finished */
                case CYCLE_1_WAIT:
                    /* drops the 1st sample into the bucket, then switches the auto state to placing1 */
                    if (autoTimer.seconds()>.5) {
                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoState = AutoState.PLACING_1;
                        autoTimer.reset();
                    }
                    break;
                /* does this code once the 1st sample is placed into the bucket */
                case PLACING_1:
                    /* starts cycle2 */
                    if (autoTimer.seconds()>.4) {
                        drivetrain.followTrajectorySequence(cycle2);
                        autoState = AutoState.CYCLE_2;

                    }
                    break;
                /* does this code if the cycle2 process is happening at the moment */
                case CYCLE_2:
                    /* switches the auto state to cycle2 WAIT if the cycle2 process is done */
                    if (drivetrain.isFinished() && Math.abs(outtake.getSlideError())<.5) {
                        autoState = AutoState.CYCLE_2_WAIT;
                        autoTimer.reset();
                    }
                    break;
                /* does this code if the cycle2 just finished */
                case CYCLE_2_WAIT:
                    /* drops the 2nd sample into the bucket, then switches the auto state to placing2 */
                    if (autoTimer.seconds()>.5) {
                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoState = AutoState.PLACING_2;
                        autoTimer.reset();
                    }
                    break;
                /* does this code once the 2nd sample is placed into the bucket */
                case PLACING_2:
                    /* starts the parking process */
                    if (autoTimer.seconds()>.4) {
                        drivetrain.followTrajectorySequence(park1);
                        autoState = AutoState.PARK1;

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
                /* does this code if the parking process is happening at the moment */
                case PARK1:
                    /* sets the auto state to finished if the robot is done parking*/
                    if (drivetrain.isFinished()) {
//                        drivetrain.followTrajectorySequence(park2);
                        autoState = AutoState.FINISHED;
                    }
                    break;
                /* unused at the moment. Ignore the PARK2 code */
                case PARK2:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.FINISHED;
                    }

            }

            masterThread.unThreadedUpdate();

            //----------------- IF INTAKE HAS JUST PLACED A SAMPLE IN THE TRANSFER -----------------
            if (intake.transfered()) {
                /* centers the claw above the transfer, grabs the sample from the transfer, and
                rotates the outtake 4bar and claw to be in a position where it can drop the sample
                into the buckets */
                transfer.setTransferState(Transfer.TransferState.CENTER);
                outtake.grabFromTransfer();
            }

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
