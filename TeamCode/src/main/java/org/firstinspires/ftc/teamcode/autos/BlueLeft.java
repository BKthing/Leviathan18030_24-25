package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous
public class BlueLeft extends LinearOpMode {

    private enum AutoState{
        PLACING_PRELOAD,
        PLACING_DELAY,
        CYCLE_1,
        PARK,
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
        outtake = new Outtake(masterThread.getData(), true, true);

//        teleopController = new TeleopController(intake, transfer, outtake, masterThread.getData());


        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer,
                intake,
                transfer,
                outtake
        );


        TrajectorySequence preload = new TrajectorySequenceBuilder(new Pose2d(16.8, 62.1, Math.toRadians(270)), RobotConstants.constraints)
                .splineToConstantHeading(new Vector2d(7, 36), Math.toRadians(270))
                .callMarker(2, () -> {
                    outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_FRONT);
                })

                .callMarkerFromEnd(.1, () -> {
                    outtake.place();
                })
                .setEndDelay(.3)
                .build();

        TrajectorySequence cycle1 = new TrajectorySequenceBuilder(preload.endPose(), RobotConstants.constraints)
                .back(2)
                .splineToConstantHeading(new Vector2d(50, 44), Math.toRadians(0))
                .callMarker(24, () -> {
                    intake.setTargetSlidePos(Intake.HorizontalSlide.AUTO_PRESET2);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                })
                .callMarkerFromEnd(8, () -> {
//                    drivetrain.setForwardComponent(.3);
                })
                .callMarkerFromEnd(1.5, () -> {
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(.5)
                .back(.1)
                .localTemporalMarker(0, () -> {
                    intake.retract();
                })
                .splineToSplineHeading(new Pose2d(55, 55, Math.toRadians(270-45)), Math.toRadians(45))
                .build();


//        drivetrain.setForwardComponent(.3);

        waitForStart();
        drivetrain.followTrajectorySequence(preload);

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());




        while ( !isStopRequested()) {

            switch (autoState) {
                case PLACING_PRELOAD:
                    if (drivetrain.isFinished()) {
//                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoTimer.reset();
                        drivetrain.followTrajectorySequence(cycle1);

                        autoState = AutoState.CYCLE_1;
                    }
                    break;
//                case PLACING_DELAY:
//                    if (autoTimer.seconds()>.2) {
//                        drivetrain.followTrajectorySequence(cycle1);
//
//                        autoState = AutoState.CYCLE_1;
//                    }
//                    break;
                case CYCLE_1:
                    if (drivetrain.isFinished()) {
                        autoState = AutoState.FINISHED;
                    }
                    break;
            }

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
