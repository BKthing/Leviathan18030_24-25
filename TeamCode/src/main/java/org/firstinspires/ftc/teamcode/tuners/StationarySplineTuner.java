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
public class StationarySplineTuner extends LinearOpMode {

    ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    CluelessConstAccelLocalizer oldLocalizer;

    MasterThread masterThread;
    Telemetry.Item loopTime;
    Telemetry.Item temporalCount;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());
        temporalCount = telemetry.addData("Temporal data", "");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        oldLocalizer = new CluelessConstAccelLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), oldLocalizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.FOLLOW_PATH);


        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer
        );

        drivetrain.setForwardComponent(0);


        TrajectorySequence spline = new TrajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)), RobotConstants.constraints)
                .splineToSplineHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))
                .build();



        waitForStart();
        drivetrain.followTrajectorySequence(spline);

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(spline.startPos());




        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
