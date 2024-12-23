package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.OldLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

@TeleOp
public class BlueTeleTest extends LinearOpMode {
    Drivetrain drivetrain;
    CluelessConstAccelLocalizer localizer;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = true;

    private List<LynxModule> allHubs;

    private Encoder perpendicularWheel, parallelWheel;


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

        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));
//        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalRight"));

        allHubs = hardwareMap.getAll(LynxModule.class);
        manualBulkReads(true);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
//                drivetrain
//                localizer
//                intake,
//                outtake
        );

//        masterThread.setParallelWheel(parallelWheel);

//        masterThread.init(hardwareMap);

        localizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();
        masterThread.clearBulkCache();

        localizer.clearDeltas();


        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();


//            parallelWheel.getCurrentPosition();

//            clearBulkCache();
//            parallelWheel.getCurrentPosition();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

    public void manualBulkReads(boolean manualReads) {
        if (manualReads) {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } else {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

}
