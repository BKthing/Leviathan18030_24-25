package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.TrajectorySequence;
import com.reefsharklibrary.pathing.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class Left1plus3Auto extends LinearOpMode {

    private enum AutoState {
        MOVING_TO_PLACE_PRELOAD,
        DROPPING_PRELOAD,
        MOVING_TO_GRAB_BLOCK_1,
        GRABBING_BLOCK_1,
        MOVING_TO_SCORE_BLOCK_1,
        SCORING_BLOCK_1,
        MOVING_TO_GRAB_BLOCK_2,
        GRABBING_BLOCK_2,
        MOVING_TO_SCORE_BLOCK_2,
        SCORING_BLOCK_2,
        MOVING_TO_GRAB_BLOCK_3,
        GRABBING_BLOCK_3,
        MOVING_TO_SCORE_BLOCK_3,
        SCORING_BLOCK_3,
        MOVING_TO_PARK,
        FINISHED
    }

    AutoState autoState = AutoState.MOVING_TO_PLACE_PRELOAD;

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    OldLocalizer oldLocalizer;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    double extensionDistance = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        oldLocalizer = new OldLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), oldLocalizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.FOLLOW_PATH);


        intake = new NewIntake(masterThread.getData(), blueAlliance, false);
        outtake = new NewOuttake(masterThread.getData(), intake, blueAlliance, false, true, true);


        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer,
                intake,
                outtake
        );

        TrajectorySequence movingToPlacePreload = new TrajectorySequenceBuilder(new Pose2d(16.8, 62.1, Math.toRadians(270)), RobotConstants.constraints)
                .splineToConstantHeading(new Vector2d(10, 33), Math.toRadians(270))
                .callMarker(2, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);
                })
                .build();

        TrajectorySequence movingToGrabBlock1 = new TrajectorySequenceBuilder(movingToPlacePreload.endPose(), RobotConstants.constraints)
                .back(2)
                .splineToConstantHeading(new Vector2d(45, 56), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, 50), Math.toRadians(270))
                .callMarkerFromEnd(11, () -> {
                    intake.setTargetSlidePos(8);
                    extensionDistance = 8;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                })
                .build();

        TrajectorySequence movingToScoreBlock1 = new TrajectorySequenceBuilder(movingToGrabBlock1.endPose(), RobotConstants.constraints)
                .splineToLineHeading(new Pose2d(56, 56, Math.toRadians(270-45)), Math.toRadians(45))
                .build();


        TrajectorySequence movingToGrabBlock2 = new TrajectorySequenceBuilder(movingToScoreBlock1.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToLineHeading(new Pose2d(52, 50, Math.toRadians(285)), Math.toRadians(285))
                .callMarkerFromEnd(8, () -> {
                    intake.setTargetSlidePos(8);
                    extensionDistance = 8;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                })
                .build();

        TrajectorySequence movingToScoreBlock2 = new TrajectorySequenceBuilder(movingToGrabBlock2.endPose(), RobotConstants.constraints)
                .splineToLineHeading(new Pose2d(55, 57, Math.toRadians(270-45)), Math.toRadians(45))
                .build();


        TrajectorySequence movingToGrabBlock3 = new TrajectorySequenceBuilder(movingToScoreBlock2.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToLineHeading(new Pose2d(52, 50, Math.toRadians(300)), Math.toRadians(300))
                .callMarkerFromEnd(8, () -> {
                    intake.setTargetSlidePos(8);
                    extensionDistance = 8;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                })
                .build();

        TrajectorySequence movingToScoreBlock3 = new TrajectorySequenceBuilder(movingToGrabBlock3.endPose(), RobotConstants.constraints)
                .splineToLineHeading(new Pose2d(53, 59, Math.toRadians(270-45)), Math.toRadians(45))
                .build();

        TrajectorySequence movingToPark = new TrajectorySequenceBuilder(movingToScoreBlock3.endPose(), RobotConstants.constraints)
                .splineToLineHeading(new Pose2d(34, 15, Math.toRadians(180)), Math.toRadians(180))
                .forward(16)
                .callMarkerFromEnd(15, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                })
                .build();



        drivetrain.followTrajectorySequence(movingToPlacePreload);

        oldLocalizer.getLocalizer().setPoseEstimate(new Pose2d(16.8, 62.1, Math.toRadians(270)));

        waitForStart();
        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(new Pose2d(16.8, 62.1, Math.toRadians(270)));

        while (!isStopRequested()) {
            masterThread.unThreadedUpdate();

            switch (autoState) {
                case MOVING_TO_PLACE_PRELOAD:
                    if (drivetrain.isFinished()) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);

                        autoTimer.reset();
                        autoState = AutoState.DROPPING_PRELOAD;
                    }
                    break;
                case DROPPING_PRELOAD:
                    if (autoTimer.seconds()>.1) {
                        drivetrain.followTrajectorySequence(movingToGrabBlock1);
                        autoState = AutoState.MOVING_TO_GRAB_BLOCK_1;
                    }
                    break;
                case MOVING_TO_GRAB_BLOCK_1:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.GRABBING_BLOCK_1;
                        autoTimer.reset();
                    }
                    break;
                case GRABBING_BLOCK_1:
                    if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock1);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_1;
                    } else if (autoTimer.seconds()>1.5) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock1);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_1;
                        intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 10 * loopTimer.seconds(), -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case MOVING_TO_SCORE_BLOCK_1:
                    if (drivetrain.isFinished()) {
                        autoTimer.reset();
                        autoState = AutoState.SCORING_BLOCK_1;
                    }
                    break;
                case SCORING_BLOCK_1:
                    if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || autoTimer.seconds()>3) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        drivetrain.followTrajectorySequence(movingToGrabBlock2);
                        autoState = AutoState.MOVING_TO_GRAB_BLOCK_2;
                    }
                    break;


                case MOVING_TO_GRAB_BLOCK_2:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.GRABBING_BLOCK_2;
                        autoTimer.reset();
                    }
                    break;
                case GRABBING_BLOCK_2:
                    if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock2);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_2;
                    } else if (autoTimer.seconds()>1.5) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock2);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_2;
                        intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 10 * loopTimer.seconds(), -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case MOVING_TO_SCORE_BLOCK_2:
                    if (drivetrain.isFinished()) {
                        autoTimer.reset();
                        autoState = AutoState.SCORING_BLOCK_2;
                    }
                    break;
                case SCORING_BLOCK_2:
                    if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || autoTimer.seconds()>3) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        drivetrain.followTrajectorySequence(movingToGrabBlock3);
                        autoState = AutoState.MOVING_TO_GRAB_BLOCK_3;
                    }
                    break;


                case MOVING_TO_GRAB_BLOCK_3:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.GRABBING_BLOCK_3;
                        autoTimer.reset();
                    }
                    break;
                case GRABBING_BLOCK_3:
                    if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock3);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_3;
                    } else if (autoTimer.seconds()>1.5) {
                        drivetrain.followTrajectorySequence(movingToScoreBlock3);
                        autoState = AutoState.MOVING_TO_SCORE_BLOCK_3;
                        intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 10 * loopTimer.seconds(), -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }
                    break;
                case MOVING_TO_SCORE_BLOCK_3:
                    if (drivetrain.isFinished()) {
                        autoTimer.reset();
                        autoState = AutoState.SCORING_BLOCK_3;
                    }
                    break;
                case SCORING_BLOCK_3:
                    if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || autoTimer.seconds()>3) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        drivetrain.followTrajectorySequence(movingToPark);

                        autoState = AutoState.MOVING_TO_PARK;
                    }
                    break;


                case MOVING_TO_PARK:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.FINISHED;
                    }
                    break;
            }

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }

    }
}
