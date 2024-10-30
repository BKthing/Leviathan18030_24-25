package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.pathing.EndpointEstimator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.EndPointEstimatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.TeleopController;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class EndPointEstimatorTuner extends LinearOpMode {
    Drivetrain drivetrain;
    OldLocalizer oldLocalizer;

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


        oldLocalizer = new OldLocalizer(masterThread.getData());

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
