package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class SplineTest extends LinearOpMode {

    ElapsedTimer autoTimer = new ElapsedTimer();

    Drivetrain drivetrain;
    OldLocalizer oldLocalizer;

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


        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer
        );

//        drivetrain.runner().setForwardComponent(0);


        TrajectorySequence spline = new TrajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)), RobotConstants.constraints)
                .splineToSplineHeading(new Pose2d(16, 48, Math.toRadians(270)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(16, 64))
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
