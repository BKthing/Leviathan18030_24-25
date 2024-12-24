package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.TrajectorySequence;
import com.reefsharklibrary.pathing.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class SplineTest2 extends LinearOpMode {

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

//        drivetrain.runner().setForwardComponent(0);


        TrajectorySequence spline = new TrajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)), RobotConstants.constraints)
                .splineToSplineHeading(new Pose2d(16, 44, Math.toRadians(270)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(16, 56))
                .setTargetEndDistance(5)
                .build();


        TrajectorySequence spline2 = new TrajectorySequenceBuilder(new Pose2d(0, 0, 0), RobotConstants.constraints)
                .back(1)
                .splineToConstantHeading(new Vector2d(-18.5, -25), Math.toRadians(180))
                .back(2)
                .build();

//        drivetrain.setForwardComponent(0);


        waitForStart();
        drivetrain.followTrajectorySequence(spline2);

        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(spline2.startPos());




        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
