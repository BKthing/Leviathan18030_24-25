package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive2.PARAMS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.SingleAction;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

@Autonomous
public class RRRight5plus0Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

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

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true);


        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );





        Action preload = drivetrain.drive.actionBuilder(new Pose2d(-5.5, 62.1, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(-4, 29), Math.toRadians(270))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(-4, 29, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterDisp(6, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .splineToSplineHeading(new Pose2d(-29, 44, Math.toRadians(240)), Math.toRadians(180))
                .turn(Math.toRadians(-100))
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .setTangent(new com.reefsharklibrary.data.Vector2d(-34.5, 40).minus(new com.reefsharklibrary.data.Vector2d(-28, 40)).getDirection())
                .lineToXSplineHeading(-34.5, Math.toRadians(250))
                .build();

        Action moveToPlaceBlock2 = drivetrain.drive.actionBuilder(new Pose2d(-34.5, 40, Math.toRadians(250)))
                .waitSeconds(.2)
                .turn(Math.toRadians(-110))
                .afterTime(.1, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .setTangent(280)
                .afterTime(0, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToLinearHeading(new Pose2d(-33, 58, Math.toRadians(270)), Math.toRadians(270), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(Math.PI))))


                .lineToY(63)
                .waitSeconds(.1)
                .build();

//        Action moveToPlaceBlock3 = drivetrain.drive.actionBuilder(new Pose2d(-38, 37, Math.toRadians(240)))
//                .setTangent(Math.toRadians(90))
//                .lineToYLinearHeading(50, Math.toRadians(170))
//                .afterTime(0, () -> {
//                    intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
//                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .splineToLinearHeading(new Pose2d(-28, 62, Math.toRadians(270)), Math.toRadians(270), new MinVelConstraint(Arrays.asList(drivetrain.drive.kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
//                        new AngularVelConstraint(Math.PI))))
//
//
//                .lineToY(64)
//                .waitSeconds(.1)
//                .build();

        Action moveToScoreSpecimen2 = drivetrain.drive.actionBuilder(new Pose2d(-33, 63, Math.toRadians(270)))
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-2, 30), Math.toRadians(305))
                .build();

        Action moveToGrabSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-2, 30, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterTime(.3, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToConstantHeading(new Vector2d(-33, 63), Math.toRadians(90))
                .waitSeconds(.1)
                .build();

        Action moveToScoreSpecimen3 = drivetrain.drive.actionBuilder(new Pose2d(-33, 63, Math.toRadians(270)))
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-4, 30), Math.toRadians(305))
                .build();

        Action moveToGrabSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-4, 30, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterTime(.3, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
                })
                .splineToConstantHeading(new Vector2d(-33, 62.5), Math.toRadians(90))
                .waitSeconds(.1)
                .build();

        Action moveToScoreSpecimen4 = drivetrain.drive.actionBuilder(new Pose2d(-33, 62.5, Math.toRadians(270)))
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-4, 30), Math.toRadians(305))
                .build();

//        Action moveToGrabSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-6, 35, Math.toRadians(270)))
//                .setTangent(Math.toRadians(90))
//                .afterTime(.3, () -> {
//                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.WAIT_DROP_BEHIND);
//                })
//                .splineToConstantHeading(new Vector2d(-28, 64), Math.toRadians(90))
//                .waitSeconds(.1)
//                .build();

//        Action moveToScoreSpecimen5 = drivetrain.drive.actionBuilder(new Pose2d(-28, 64, Math.toRadians(270)))
//                .waitSeconds(1)
//                .setTangent(Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(-4, 35), Math.toRadians(270))
//                .build();

        Action moveToPark = drivetrain.drive.actionBuilder(new Pose2d(-4, 30, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .afterTime(.6, () -> {
                    intake.setTargetSlidePos(18.5);
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                })
                .splineToSplineHeading(new Pose2d(-22, 50, Math.toRadians(155)), Math.toRadians(165))
                .build();


        waitForStart();

        drivetrain.drive.setPoseEstimate(new  Pose2d(-5.5, 62.1, Math.toRadians(270)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabBlock1,
                new SingleAction(() -> intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE)),
                moveToPlaceBlock2,
//                new SingleAction(() -> intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE)),
//                moveToPlaceBlock3,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen2,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen3,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen3,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabSpecimen4,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
                moveToScoreSpecimen4,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
//                moveToGrabSpecimen5,
//                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.CLOSED)),
//                moveToScoreSpecimen5,
//                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToPark
                ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

}
