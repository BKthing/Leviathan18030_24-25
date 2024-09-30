package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class Intake extends SubSystem {

    public enum IntakeState {
        EXTENDING,
        EXTENDING_INTAKE_DOWN,
        INTAKING,
        RETRACTING,
        TRANSFERING,
        RESTING
    }

    IntakeState intakeState = IntakeState.RESTING;

    public enum HorizontalSlide {
        //18.9 max
        IN(0),
        AUTO_PRESET1(12),
        AUTO_PRESET2(4),
        CLOSE(8),
        MEDIUM(12),
        FAR(16);

        public final double length;
        HorizontalSlide(double length) {this.length = length;}
    }

    boolean changedTargetSlidePos = false;
    double targetSlidePos;
    double newTargetSlidePos;

    double slidePos;//inches
    double prevSlideError = 0;

    double slideI = 0;

    int slideTicks = 0;

    Encoder horizontalEncoder;

    DcMotorEx horizontalLeftMotor, horizontalRightMotor;

    public enum IntakePos {
        UP(.3),
        PARTIAL_UP(.4),
        CLEAR_BAR(.62),
        DOWN(.7);

        public final double pos;
        IntakePos(double pos) {this.pos = pos;}
    }
    IntakePos intakePos = IntakePos.PARTIAL_UP;
    IntakePos newIntakePos = IntakePos.PARTIAL_UP;

    boolean changedIntakePos = false;
    boolean updateIntakePos = true;

    boolean changedIntakeSpeed = false;
    double targetIntakeSpeed = 0;
    double actualIntakeSpeed = 0;
    double newIntakeSpeed = 0;


    Servo leftIntakeServo, rightIntakeServo, intakeServo;

    ElapsedTimer slideTimer = new ElapsedTimer();

    public Intake(SubSystemData data) {
        super(data);

        horizontalEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));

        horizontalLeftMotor = hardwareMap.get(DcMotorEx.class, "horizontalLeft");
        horizontalRightMotor = hardwareMap.get(DcMotorEx.class, "horizontalRight");

        horizontalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");

        leftIntakeServo.setDirection(Servo.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");


        //initiating slide encoder
        slidePos = ticksToInches(horizontalEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = ticksToInches(horizontalEncoder.getCurrentPosition());
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;


    }

    @Override
    public void priorityData() {
        slideTicks = horizontalEncoder.getCurrentPosition();

        targetSlidePos = newTargetSlidePos;

        if (intakePos != newIntakePos) {
            intakePos = newIntakePos;
            updateIntakePos = true;
        }

        targetIntakeSpeed = newIntakeSpeed;
    }

    @Override
    public void loop() {
        //slide PID
        slidePos = ticksToInches(slideTicks);
        //limits max time
        double elapsedTime = Math.min(slideTimer.seconds(), .5);

        //pid control
        double error = targetSlidePos - slidePos;
        double absError = Math.abs(error);

        double p, d = 0;

        slideI += error*elapsedTime;

        //Checks if error is in acceptable amounts
        if (absError<.1) {
            p = .04;
        } else if (absError>4) {
            //Slides set to max power
            p = Math.signum(error);
        } else {//if (error<4 but error>.1)
            p = error*.1;
            d = ((error - prevSlideError) / elapsedTime) * .007;//.007
        }

        double motorPower = p + slideI - d;
        slideTimer.reset();
        prevSlideError = error;

        hardwareQueue.add(() -> horizontalLeftMotor.setPower(motorPower));
        hardwareQueue.add(() -> horizontalRightMotor.setPower(motorPower));




        //intake code
        if (updateIntakePos) {
            hardwareQueue.add(() -> leftIntakeServo.setPosition(intakePos.pos));
            hardwareQueue.add(() -> rightIntakeServo.setPosition(intakePos.pos));

            updateIntakePos = false;
        }

        if (Math.abs(targetIntakeSpeed-actualIntakeSpeed)>.05) {
            actualIntakeSpeed = targetIntakeSpeed;
            hardwareQueue.add(() -> intakeServo.setPosition(actualIntakeSpeed));
        }

        switch (intakeState) {
            case EXTENDING:
                if (absError<.5)
                break;
            case INTAKING:

                break;
            case RETRACTING:

                break;
            case TRANSFERING:

                break;
        }
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {


        return super.dashboard(packet);
    }

    private double ticksToInches(int ticks) {
        return (ticks/145.1)*4.72;
    }

    public void setTargetSlidePos(double targetPos) {
        changedTargetSlidePos = true;
        newTargetSlidePos = MathUtil.clip(targetPos, 0, 18.7);//18.9 is max .2 to be safe
    }

    public void setTargetSlidePos(HorizontalSlide targetPos) {
        changedTargetSlidePos = true;
        newTargetSlidePos = targetPos.length;
    }

    public void setIntakePos(IntakePos intakePos) {
        changedIntakePos = true;
        newIntakePos = intakePos;
    }

    public void setTargetIntakeSpeed(double intakeSpeed) {
        changedIntakeSpeed = true;
        newIntakeSpeed = intakeSpeed;
    }
}
