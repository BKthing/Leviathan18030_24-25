package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive2;
import org.firstinspires.ftc.teamcode.subsystems.CluelessConstAccelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

@Autonomous
public class AutoTest extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = true;

    private List<LynxModule> allHubs;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private DcMotorEx perpendicularWheel, parallelWheel;



    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTimer loopTimer = new ElapsedTimer();
        loopTime = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        perpendicularWheel = hardwareMap.get(DcMotorEx.class, "verticalRight");
        parallelWheel = hardwareMap.get(DcMotorEx.class, "bl");

        drivetrain = new NewDrivetrain(masterThread.getData(), parallelWheel, perpendicularWheel);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);


        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, false);


        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, false);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );



        drivetrain.drive.pose = new com.acmerobotics.roadrunner.Pose2d(64, 14, Math.toRadians(180));
        drivetrain.roadRunnerPose = new com.acmerobotics.roadrunner.Pose2d(64, 14, Math.toRadians(180));


        waitForStart();
        masterThread.clearBulkCache();


        while ( !isStopRequested()) {
            drivetrain.followPath(
                    drivetrain.drive.actionBuilder(new Pose2d(64, 14, Math.toRadians(180)))
                            .lineToX(20)
                            .build()
            );
            Actions.runBlocking(
                    drivetrain.drive.actionBuilder(new Pose2d(64, 14, Math.toRadians(180)))
//                        .setTangent(Math.toRadians(180))
//                        .splineToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(0, 0), Math.toRadians(180))
//                        .setTangent(Math.toRadians(0))
//                        .splineToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(64, 14), Math.toRadians(0))
//                        .build());
                            .lineToX(20)
                            .lineToX(0)
                            .build());
//            if (drivetrain.isFinished()) {
//                Action testPath = drivetrain.drive.actionBuilder(new com.acmerobotics.roadrunner.Pose2d(64, 14, Math.toRadians(180)))
//                        .setTangent(Math.toRadians(180))
//                        .splineToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(0, 0), Math.toRadians(180))
//                        .setTangent(Math.toRadians(0))
//                        .splineToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(64, 14), Math.toRadians(0))
//                        .build();
//                drivetrain.followPath(testPath);
            }

            masterThread.unThreadedUpdate();
//            drive.updatePoseEstimate();


            parallelWheel.getCurrentPosition();
            perpendicularWheel.getCurrentPosition();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

