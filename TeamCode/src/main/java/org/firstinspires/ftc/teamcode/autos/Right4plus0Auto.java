package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.TrajectorySequence;
import com.reefsharklibrary.pathing.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class Right4plus0Auto extends LinearOpMode {

    private enum AutoState {
        DRIVING_TO_PLACE_PRELOAD,
        RELEASING_PRELOAD,
        MOVING_TO_GRAB_BLOCK_1,
        GRABBING_BLOCK_1,
//        MOVING_TO_DROP_BLOCK_1,
        MOVING_TO_GRAB_BLOCK_2,
        GRABBING_BLOCK_2,
//        MOVING_TO_DROP_BLOCK_2,
        MOVING_TO_GRAB_BLOCK_3,
        GRABBING_BLOCK_3,
//        MOVING_TO_DROP_BLOCK_3,

        MOVING_TO_GRAB_SPECIMEN_1,
        GRABBING_SPECIMEN_1,
        MOVING_TO_PLACE_SPECIMEN_1,
        PLACING_SPECIMEN_1,

        MOVING_TO_GRAB_SPECIMEN_2,
        GRABBING_SPECIMEN_2,
        MOVING_TO_PLACE_SPECIMEN_2,
        PLACING_SPECIMEN_2,

        MOVING_TO_GRAB_SPECIMEN_3,
        GRABBING_SPECIMEN_3,
        MOVING_TO_PLACE_SPECIMEN_3,
        PLACING_SPECIMEN_3,

        MOVING_TO_GRAB_SPECIMEN_4,
        GRABBING_SPECIMEN_4,
        MOVING_TO_PLACE_SPECIMEN_4,
        PLACING_SPECIMEN_4,

        PARKING,
        TRANSITION,
        FINISHED
    }

    AutoState autoState = AutoState.DRIVING_TO_PLACE_PRELOAD;

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    CluelessConstAccelLocalizer localizer;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    double extensionDistance = 0;

    private Encoder perpendicularWheel, parallelWheel, verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        localizer = new CluelessConstAccelLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), localizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.FOLLOW_PATH);

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));
        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true);
        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true);


        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                localizer,
                intake,
                outtake
        );


        TrajectorySequence preload = new TrajectorySequenceBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)), RobotConstants.constraints)
                .splineToConstantHeading(new Vector2d(-1, 30.5), Math.toRadians(270))
                .callMarker(3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .build();

        TrajectorySequence movingToGrabBlock1 = new TrajectorySequenceBuilder(preload.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToSplineHeading(new Pose2d(-28, 44, Math.toRadians(240)), Math.toRadians(180))
                .callMarkerFromEnd(5, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .build();


        Pose2d movingToDropBlock1EndPose = new Pose2d(-30, 47, Math.toRadians(160));

        TrajectorySequence movingToGrabBlock2 = new TrajectorySequenceBuilder(movingToDropBlock1EndPose, RobotConstants.constraints)
                .lineToSplineHeading(new Pose2d(-34, 46, Math.toRadians(240)))
                .callMarkerFromEnd(1, () -> {
                    intake.setTargetSlidePos(18.5);
                })
                .build();

        TrajectorySequence movingToDropBlock1 = new TrajectorySequenceBuilder(movingToGrabBlock1.endPose(), RobotConstants.constraints)
                .lineToSplineHeading(movingToDropBlock1EndPose)
                .callMarkerFromEnd(0.1, () -> {
                    drivetrain.followTrajectorySequence(movingToGrabBlock2);
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_TO_AUTO_HEIGHT);
                    intake.setTargetSlidePos(12);

                    autoState = AutoState.MOVING_TO_GRAB_BLOCK_2;
                })
                .build();



        Pose2d movingToDropBlock2EndPose = new Pose2d(-36, 47, Math.toRadians(160));

        TrajectorySequence movingToGrabBlock3 = new TrajectorySequenceBuilder(movingToDropBlock2EndPose, RobotConstants.constraints)
                .lineToSplineHeading(new Pose2d(-38.5, 43, Math.toRadians(235)))
                .callMarkerFromEnd(2, () -> {
                    intake.setTargetSlidePos(18.5);
                })
                .build();

        TrajectorySequence movingToDropBlock2 = new TrajectorySequenceBuilder(movingToGrabBlock2.endPose(), RobotConstants.constraints)
                .lineToSplineHeading(movingToDropBlock2EndPose)
                .callMarkerFromEnd(0.1, () -> {
                    drivetrain.followTrajectorySequence(movingToGrabBlock3);
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_TO_AUTO_HEIGHT);
                    intake.setTargetSlidePos(12);

                    autoState = AutoState.MOVING_TO_GRAB_BLOCK_3;
                })
                .build();



        Pose2d movingToDropBlock3EndPose = new Pose2d(-36, 47, Math.toRadians(150));

        TrajectorySequence movingToGrabSpecimen1 = new TrajectorySequenceBuilder(movingToDropBlock3EndPose, RobotConstants.constraints)
                .lineToSplineHeading(new Pose2d(-28, 63, Math.toRadians(270)))
                .setEndDelay(.5)
                .back(4)
                .build();

        TrajectorySequence movingToDropBlock3 = new TrajectorySequenceBuilder(movingToGrabBlock2.endPose(), RobotConstants.constraints)
                .lineToSplineHeading(movingToDropBlock3EndPose)
                .callMarkerFromEnd(0.1, () -> {
                    drivetrain.followTrajectorySequence(movingToGrabSpecimen1);
                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);

                    autoState = AutoState.MOVING_TO_GRAB_SPECIMEN_1;
                })
                .build();




        TrajectorySequence movingToPlaceSpecimen1 = new TrajectorySequenceBuilder(movingToGrabSpecimen1.endPose(), RobotConstants.constraints)
                .forward(1)
                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(270))
                .build();



        TrajectorySequence movingToGrabSpecimen2 = new TrajectorySequenceBuilder(movingToPlaceSpecimen1.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToConstantHeading(new Vector2d(-30, 63), Math.toRadians(90))
                .callMarker(5, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .setEndDelay(.5)
                .back(4)
                .build();

        TrajectorySequence movingToPlaceSpecimen2 = new TrajectorySequenceBuilder(movingToGrabSpecimen2.endPose(), RobotConstants.constraints)
                .forward(1)
                .splineToConstantHeading(new Vector2d(-.5, 35), Math.toRadians(270))
                .build();



        TrajectorySequence movingToGrabSpecimen3 = new TrajectorySequenceBuilder(movingToPlaceSpecimen2.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToConstantHeading(new Vector2d(-30, 63), Math.toRadians(90))
                .callMarker(5, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .setEndDelay(.5)
                .back(4)
                .build();

        TrajectorySequence movingToPlaceSpecimen3 = new TrajectorySequenceBuilder(movingToGrabSpecimen3.endPose(), RobotConstants.constraints)
                .callMarker(0, () -> {
                    drivetrain.setForwardComponent(100);
                })
                .forward(1)
                .splineToConstantHeading(new Vector2d(-1, 35), Math.toRadians(270))
                .build();



//        TrajectorySequence movingToGrabSpecimen4 = new TrajectorySequenceBuilder(movingToPlaceSpecimen3.endPose(), RobotConstants.constraints)
//                .back(1)
//                .splineToConstantHeading(new Vector2d(-30, 63), Math.toRadians(90))
//                .callMarker(5, () -> {
//                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .back(2)
//                .build();

//        TrajectorySequence movingToPlaceSpecimen4 = new TrajectorySequenceBuilder(movingToGrabSpecimen4.endPose(), RobotConstants.constraints)
//                .forward(1)
//                .splineToConstantHeading(new Vector2d(-4, 36), Math.toRadians(270))
//                .build();


        TrajectorySequence park = new TrajectorySequenceBuilder(movingToPlaceSpecimen3.endPose(), RobotConstants.constraints)
                .back(1)
                .splineToLineHeading(new Pose2d(-22, 50, Math.toRadians(155)), Math.toRadians(170))
                .callMarker(8, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .build();


        drivetrain.followTrajectorySequence(preload);

        localizer.getLocalizer().setPoseEstimate(new Pose2d(-5.5, 62.1, Math.toRadians(270)));

        waitForStart();
        masterThread.clearBulkCache();

        localizer.clearDeltas();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);

        while (!isStopRequested()) {
            masterThread.unThreadedUpdate();

            switch (autoState) {
                case DRIVING_TO_PLACE_PRELOAD:
                    if (drivetrain.isFinished()) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);

                        autoTimer.reset();

                        autoState = AutoState.RELEASING_PRELOAD;
                    }
                    break;
                case RELEASING_PRELOAD:
                    if (autoTimer.seconds()>.1) {
//                        drivetrain.setForwardComponent(.6);
                        drivetrain.followTrajectorySequence(movingToGrabBlock1);
                        autoState = AutoState.MOVING_TO_GRAB_BLOCK_1;
                    }
                    break;


                case MOVING_TO_GRAB_BLOCK_1:
                    if (drivetrain.isFinished()) {
//                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);

                        autoTimer.reset();

                        autoState = AutoState.GRABBING_BLOCK_1;
                    }
                        break;
                case GRABBING_BLOCK_1:
                    if (autoTimer.seconds()>.15) {
                        drivetrain.followTrajectorySequence(movingToDropBlock1);

                        autoState = AutoState.TRANSITION;
                    }
                    break;


                case MOVING_TO_GRAB_BLOCK_2:
                    if (drivetrain.isFinished()) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);

                        autoTimer.reset();

                        autoState = AutoState.GRABBING_BLOCK_2;
                    }
                    break;
                case GRABBING_BLOCK_2:
                    if (autoTimer.seconds()>.15) {
                        drivetrain.followTrajectorySequence(movingToDropBlock2);

                        autoState = AutoState.TRANSITION;
                    }
                    break;


                case MOVING_TO_GRAB_BLOCK_3:
                    if (drivetrain.isFinished()) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);

                        autoTimer.reset();

                        autoState = AutoState.GRABBING_BLOCK_3;
                    }
                    break;
                case GRABBING_BLOCK_3:
                    if (autoTimer.seconds()>.15) {
                        drivetrain.followTrajectorySequence(movingToDropBlock3);
                        autoState = AutoState.TRANSITION;
                    }
                    break;


                case MOVING_TO_GRAB_SPECIMEN_1:
                    if (drivetrain.isFinished() && autoTimer.seconds() > .5) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                        autoTimer.reset();

                        autoState = AutoState.GRABBING_SPECIMEN_1;
                    }
                    break;
                case GRABBING_SPECIMEN_1:
                    if (autoTimer.seconds()>.5) {
                        drivetrain.followTrajectorySequence(movingToPlaceSpecimen1);

                        autoState = AutoState.MOVING_TO_PLACE_SPECIMEN_1;
                    }
                    break;
                case MOVING_TO_PLACE_SPECIMEN_1:
                    if (drivetrain.isFinished()) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        autoTimer.reset();

                        autoState = AutoState.PLACING_SPECIMEN_1;
                    }
                    break;
                case PLACING_SPECIMEN_1:
                    if (autoTimer.seconds()>.1) {
                        drivetrain.followTrajectorySequence(movingToGrabSpecimen2);
                        autoState = AutoState.MOVING_TO_GRAB_SPECIMEN_2;
                    }
                    break;


                case MOVING_TO_GRAB_SPECIMEN_2:
                    if (drivetrain.isFinished() && autoTimer.seconds() > .5) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                        autoTimer.reset();

                        autoState = AutoState.GRABBING_SPECIMEN_2;
                    }
                    break;
                case GRABBING_SPECIMEN_2:
                    if (autoTimer.seconds()>.5) {
                        drivetrain.followTrajectorySequence(movingToPlaceSpecimen2);

                        autoState = AutoState.MOVING_TO_PLACE_SPECIMEN_2;
                    }
                    break;
                case MOVING_TO_PLACE_SPECIMEN_2:
                    if (drivetrain.isFinished()) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        autoTimer.reset();

                        autoState = AutoState.PLACING_SPECIMEN_2;
                    }
                    break;
                case PLACING_SPECIMEN_2:
                    if (autoTimer.seconds()>.1) {
                        drivetrain.followTrajectorySequence(movingToGrabSpecimen3);
                        autoState = AutoState.MOVING_TO_GRAB_SPECIMEN_3;
                    }
                    break;


                case MOVING_TO_GRAB_SPECIMEN_3:
                    if (drivetrain.isFinished() && autoTimer.seconds() > .5) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
                        autoTimer.reset();

                        autoState = AutoState.GRABBING_SPECIMEN_3;
                    }
                    break;
                case GRABBING_SPECIMEN_3:
                    if (autoTimer.seconds()>.3) {
                        drivetrain.followTrajectorySequence(movingToPlaceSpecimen3);

                        autoState = AutoState.MOVING_TO_PLACE_SPECIMEN_3;
                    }
                    break;
                case MOVING_TO_PLACE_SPECIMEN_3:
                    if (drivetrain.isFinished()) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        autoTimer.reset();

                        autoState = AutoState.PLACING_SPECIMEN_3;
                    }
                    break;
                case PLACING_SPECIMEN_3:
                    if (autoTimer.seconds()>.1) {
                        drivetrain.followTrajectorySequence(park);
                        autoState = AutoState.PARKING;
                    }
                    break;


//                case MOVING_TO_GRAB_SPECIMEN_4:
//                    if (drivetrain.isFinished()) {
//                        outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED);
//                        autoTimer.reset();
//
//                        autoState = AutoState.GRABBING_SPECIMEN_4;
//                    }
//                    break;
//                case GRABBING_SPECIMEN_4:
//                    if (autoTimer.seconds()>.3) {
//                        drivetrain.followTrajectorySequence(movingToPlaceSpecimen4);
//
//                        autoState = AutoState.MOVING_TO_PLACE_SPECIMEN_4;
//                    }
//                    break;
//                case MOVING_TO_PLACE_SPECIMEN_4:
//                    if (drivetrain.isFinished()) {
//                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
//                        autoTimer.reset();
//
//                        autoState = AutoState.PLACING_SPECIMEN_4;
//                    }
//                    break;
//                case PLACING_SPECIMEN_4:
//                    if (autoTimer.seconds()>.1) {
//                        drivetrain.followTrajectorySequence(park);
//                        autoState = AutoState.PARKING;
//                    }
//                    break;


                case PARKING:
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
