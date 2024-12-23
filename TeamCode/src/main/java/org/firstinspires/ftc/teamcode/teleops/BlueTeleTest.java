package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    private Encoder perpendicularWheel, parallelWheel, verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);


        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));
        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "bl"));

        localizer = new CluelessConstAccelLocalizer(masterThread.getData());
        localizer.initSensors(perpendicularWheel, parallelWheel);


        drivetrain = new Drivetrain(masterThread.getData(), localizer.getLocalizer());
        drivetrain.setDriveState(Drivetrain.DriveState.DRIVER_CONTROL);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, true, false);


        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, true, true, true, false);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                localizer,
                drivetrain,
                intake,
                outtake
        );

        localizer.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();
        masterThread.clearBulkCache();

        localizer.clearDeltas();


        while ( !isStopRequested()) {
            masterThread.unThreadedUpdate();

            parallelWheel.getCurrentPosition();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

}
