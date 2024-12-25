package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.depricated.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class CluelessLocalizerTest extends LinearOpMode {
    Drivetrain drivetrain;
    OldLocalizer oldLocalizer;
    //    TwoWheel twoWheel;
//    Intake intake;
//    Outtake outtake;
//    TeleopController teleopController;
//    Transfer transfer;
    MasterThread masterThread;
    Telemetry.Item loopTime;
    Telemetry.Item temporalCount;
    Telemetry.Item clulessTelem;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());
        temporalCount = telemetry.addData("Temporal data", "");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        CluelessConstAccelLocalizer cluelessConstAccelLocalizer= new CluelessConstAccelLocalizer(masterThread.getData());
        oldLocalizer = new OldLocalizer(masterThread.getData());




        drivetrain = new Drivetrain(masterThread.getData(), cluelessConstAccelLocalizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.DRIVER_CONTROL);

//        twoWheel = new TwoWheel(-5.98040, -2.56890);


        masterThread.addSubSystems(
                drivetrain,
                oldLocalizer,
                cluelessConstAccelLocalizer

        );


        oldLocalizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        cluelessConstAccelLocalizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        waitForStart();

        masterThread.clearBulkCache();

        oldLocalizer.clearDeltas();
        cluelessConstAccelLocalizer.clearDeltas();


        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
