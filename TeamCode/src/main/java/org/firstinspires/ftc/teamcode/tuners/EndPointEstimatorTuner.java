package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.depricated.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.EndPointEstimatorSubsystem;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class EndPointEstimatorTuner extends LinearOpMode {
    Drivetrain drivetrain;
    CluelessConstAccelLocalizer oldLocalizer;

    EndPointEstimatorSubsystem endPointEstimatorSubsystem;
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
        drivetrain.setDriveState(Drivetrain.DriveState.DRIVER_CONTROL);

        endPointEstimatorSubsystem = new EndPointEstimatorSubsystem(masterThread.getData(), oldLocalizer);



        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer,
                endPointEstimatorSubsystem
        );


        waitForStart();
        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
