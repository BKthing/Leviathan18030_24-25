package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.SingleAction;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

import java.util.List;

@Autonomous
public class RRLeft1plus3Auto extends LinearOpMode {
    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    MasterThread masterThread;
    Telemetry.Item loopTime;

    Boolean blueAlliance = null;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    private DcMotorEx perpendicularWheel, parallelWheel;

    double extensionDistance = 0;

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    @Override
    public void runOpMode() throws InterruptedException {
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

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, drivetrain.roadRunnerLocalizer);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake
        );

        Action intakeBlock = new IntakeBlock();


        Action preload = drivetrain.drive.actionBuilder(new Pose2d(16.8, 62.1, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .splineToConstantHeading(new Vector2d(5, 29), Math.toRadians(270))
                .build();

        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(5, 29, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .lineToY(31)
                .splineToConstantHeading(new Vector2d(45, 56), Math.toRadians(0))
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(7);
                    extensionDistance = 7;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(48, 50), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(48, 50, Math.toRadians(270)))
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)), Math.toRadians(45))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(225)))
                .waitSeconds(.1)
                .setTangent(270)
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(10);
                    extensionDistance = 10;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToLinearHeading(new Pose2d(52, 51.5, Math.toRadians(280)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock2 = drivetrain.drive.actionBuilder(new Pose2d(48, 51.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(225)))
                .waitSeconds(.1)
                .setTangent(270)
                .afterTime(0, () -> {
                    intake.setTargetSlidePos(11);
                    extensionDistance = 11;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToLinearHeading(new Pose2d(53, 51.5, Math.toRadians(293)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(53, 47.5, Math.toRadians(303)))
                .setTangent(Math.toRadians(105))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(225)), Math.toRadians(90))
                .build();

        Action park = drivetrain.drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(225)))
                .afterTime(.5, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                })
                .afterTime(1, () -> {
                    outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                })
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(35, 13, Math.toRadians(180)), Math.toRadians(180))
                .lineToX(22)
                .build();

        waitForStart();
        drivetrain.drive.pose = new Pose2d(16.8, 62.1, Math.toRadians(270));
        drivetrain.roadRunnerPose = new Pose2d(16.8, 62.1, Math.toRadians(270));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_FRONT);

        drivetrain.followPath(new SequentialAction(
                preload,
                new SingleAction(() -> outtake.toClawPosition(NewOuttake.ClawPosition.OPEN)),
                moveToGrabBlock1,
                intakeBlock,
                moveToScoreBlock1,
                new ScoreBlock(),
                moveToGrabBlock2,
                intakeBlock,
                moveToScoreBlock2,
                new ScoreBlock(),
                moveToGrabBlock3,
                intakeBlock,
                moveToScoreBlock3,
                new ScoreBlock(),
                park
                ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }
    }

    public class IntakeBlock implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                return false;
            } else if (autoTimer.seconds()>2) {
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
                return false;
            } else {
                extensionDistance = MathUtil.clip(extensionDistance + 20 * loopTimer.seconds(), -.5, 18.5);
                intake.setTargetSlidePos(extensionDistance);
                return true;
            }
        }
    }

    public class ScoreBlock implements Action {
        private boolean firstLoop = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                if (intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING) {
                    intake.setIntakingState(NewIntake.IntakingState.START_EJECTING_PARTIAL_GRAB);
                    return false;
                } else {
                    firstLoop = false;
                    autoTimer.reset();
                    return true;
                }
            } else {
                if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer() || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING) {
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

}
