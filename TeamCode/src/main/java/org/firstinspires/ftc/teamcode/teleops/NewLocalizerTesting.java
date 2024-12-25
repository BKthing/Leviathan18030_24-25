package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.ConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.depricated.Drivetrain;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@TeleOp
public class NewLocalizerTesting extends LinearOpMode {
    Drivetrain drivetrain;
    CluelessConstAccelLocalizer oldLocalizer;
    ConstAccelLocalizer localizer;
//    TwoWheel twoWheel;
//    Intake intake;
//    Outtake outtake;
//    TeleopController teleopController;
//    Transfer transfer;
    MasterThread masterThread;
    Telemetry.Item loopTime;
    Telemetry.Item temporalCount;
    Telemetry.Item twoWheelTelem;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());
        temporalCount = telemetry.addData("Temporal data", "");
        twoWheelTelem = telemetry.addData("Two Wheel points:", "");

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        oldLocalizer = new CluelessConstAccelLocalizer(masterThread.getData());

        localizer = new ConstAccelLocalizer(masterThread.getData());

        drivetrain = new Drivetrain(masterThread.getData(), oldLocalizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.DRIVER_CONTROL);

//        twoWheel = new TwoWheel(-5.98040, -2.56890);


        masterThread.addSubSystems(
                drivetrain,
//                oldLocalizer,
                localizer

        );


        waitForStart();
        masterThread.unThreadedUpdate();
        oldLocalizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();
            twoWheelTelem.setValue("Time X: " + localizer.getTwoWheel().getDeltaX().getVal()  + " Time Y: "
                    + localizer.getTwoWheel().getDeltaY().getVal() + " Time H: " + localizer.getTwoWheel().getDeltaHeading().getVal());

            loopTime.setValue(loopTimer.milliSeconds());
            loopTimer.reset();
        }
    }
}
