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


        TrajectorySequence preload = new TrajectorySequenceBuilder(new Pose2d(16.8, 62.1, Math.toRadians(90)), RobotConstants.constraints)
                .splineToConstantHeading(new Vector2d(13, 36), Math.toRadians(270))
                .callMarker(2, () -> {
                    outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_BAR);
                })
                .callMarker(62.1-36-.1, () -> {
                    outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_BAR.length-.5);
                })
                .setEndDelay(.3)
                .build();

        TrajectorySequence cycle1 = new TrajectorySequenceBuilder(preload.endPose(), RobotConstants.constraints)
                .back(2)
                .splineToConstantHeading(new Vector2d(60, 44), Math.toRadians(0))
                .callMarker(20, () -> {
                    intake.setTargetSlidePos(Intake.HorizontalSlide.CLOSE);
                    intake.setTargetIntakePos(Intake.IntakePos.DOWN);
                })
                .callMarkerFromEnd(1.5, () -> {
                    intake.setTargetIntakeSpeed(1);
                })
                .setEndDelay(.5)
                .back(.1)
                .localTemporalMarker(0, () -> {
                    intake.retract();
                })
                .splineToSplineHeading(new Pose2d(65, 65, Math.toRadians(-45)), Math.toRadians(135))
                .build();


        waitForStart();
        drivetrain.runner().followTrajectorySequence(preload);

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(preload.startPos());




        while ( !isStopRequested()) {

            switch (autoState) {
                case PLACING_PRELOAD:
                    if (drivetrain.runner().isFinished()) {
                        outtake.setClawPosition(Outtake.ClawPosition.OPEN);

                        autoTimer.reset();
                        autoState = AutoState.PLACING_DELAY;
                    }
                    break;
                case PLACING_DELAY:
                    if (autoTimer.seconds()>.2) {
                        drivetrain.runner().followTrajectorySequence(cycle1);

                        autoState = AutoState.CYCLE_1;
                    }
                    break;
                case CYCLE_1:
                    if (drivetrain.runner().isFinished()) {
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
