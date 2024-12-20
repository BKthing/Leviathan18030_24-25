package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class BlueTeleTest extends LinearOpMode {
    Drivetrain drivetrain;
    CluelessConstAccelLocalizer localizer;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = true;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        localizer = new CluelessConstAccelLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), localizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.DRIVER_CONTROL);


        intake = new NewIntake(masterThread.getData(), blueAlliance, true, false);
        outtake = new NewOuttake(masterThread.getData(), intake, blueAlliance, true, true, true, false);


        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                localizer,
                intake,
                outtake
        );

        localizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();
        masterThread.clearBulkCache();

        localizer.clearDeltas();


        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
