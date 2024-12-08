package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.TrajectorySequence;
import com.reefsharklibrary.pathing.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class BackAndForthTuner extends LinearOpMode {

    ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    CluelessConstAccelLocalizer localizer;

    MasterThread masterThread;
    Telemetry.Item loopTime;
    Telemetry.Item temporalCount;

    boolean forwardBol = true;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());
        temporalCount = telemetry.addData("Temporal data", "");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        localizer = new CluelessConstAccelLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), localizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.FOLLOW_PATH);


        masterThread.addSubSystems(
                drivetrain,
                localizer
        );


        TrajectorySequence forward = new TrajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)), RobotConstants.constraints)
                .forward(30)
                .build();

        TrajectorySequence backward = new TrajectorySequenceBuilder(forward.endPose(), RobotConstants.constraints)
                .back(30)
                .build();


        waitForStart();
        drivetrain.followTrajectorySequence(forward);

        masterThread.unThreadedUpdate();
        localizer.getLocalizer().setPoseEstimate(forward.startPos());




        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            if (drivetrain.isFinished()) {
                if (forwardBol) {
                    drivetrain.followTrajectorySequence(backward);
                    forwardBol = false;
                } else {
                    drivetrain.followTrajectorySequence(forward);
                    forwardBol = true;
                }
            }

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
