package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class Intake extends SubSystem {

    public enum IntakeState {
        EXTENDING,
        DROP_INTAKE,
        INTAKING,
        RETRACTING,
        TRANSFERING,
        RESTING
    }

    IntakeState intakeState = IntakeState.RESTING;

    IntakeState newIntakeState = IntakeState.RESTING;

    boolean changedIntakeState = false;

    ElapsedTimer intakeTimer = new ElapsedTimer();

    Telemetry.Item slidePosTelem;

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
    double targetSlidePos = 0;
    double newTargetSlidePos;

    double prevTargetSlidePos = 0;

    double slidePos;//inches
    double prevSlideError = 0;

    double slideI = 0;

    int slideTicks = 0;

    Encoder horizontalSlideEncoder;

    DcMotorEx horizontalLeftMotor, horizontalRightMotor;

    public enum IntakePos {
        UP(.7),
        PARTIAL_UP(.4),
        CLEAR_BAR(.62),
        DOWN(.1);

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


    private Servo leftIntakeServo, rightIntakeServo;
    private CRServo intakeServo;

    ElapsedTimer slideTimer = new ElapsedTimer();

    boolean holdingSample = false;

    public Intake(SubSystemData data) {
        super(data);

        //Motors
        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));
        horizontalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        horizontalLeftMotor = hardwareMap.get(DcMotorEx.class, "horizontalLeft"); // control hub 0
        horizontalRightMotor = hardwareMap.get(DcMotorEx.class, "horizontalRight"); // control hub 1

        horizontalLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Servos
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");


        rightIntakeServo.setDirection(Servo.Direction.REVERSE);


        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServo.getPortNumber();
        horizontalLeftMotor.getCurrent(CurrentUnit.AMPS);


        //initiating slide encoder
        slidePos = ticksToInches(horizontalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        slidePosTelem = telemetry.addData("Slide position", "");


    }

    @Override
    public void priorityData() {
        if (changedIntakeState) {
            intakeState = newIntakeState;
            changedIntakeState = false;
        }

        slideTicks = horizontalSlideEncoder.getCurrentPosition();

        if(changedTargetSlidePos) {
            targetSlidePos = newTargetSlidePos;
            changedTargetSlidePos = false;
        }

        if (changedIntakePos && intakePos != newIntakePos) {
            intakePos = newIntakePos;
            updateIntakePos = true;
            changedIntakePos = false;
        }

        if (changedIntakeSpeed) {
            targetIntakeSpeed = newIntakeSpeed;
            changedIntakeSpeed = false;
        }

        prevTargetSlidePos = targetSlidePos;
    }

    //trying to add to things to hardware queue asap so their more time for it to be called
    @Override
    public void loop() {

        //intake code
        if (updateIntakePos) {
            hardwareQueue.add(() -> leftIntakeServo.setPosition(intakePos.pos + .009));
            hardwareQueue.add(() -> rightIntakeServo.setPosition(intakePos.pos));

            updateIntakePos = false;
        }

        if (Math.abs(targetIntakeSpeed-actualIntakeSpeed)>.05) {
            actualIntakeSpeed = targetIntakeSpeed;
            hardwareQueue.add(() -> intakeServo.setPower(actualIntakeSpeed));
        }


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
            d = ((error - prevSlideError) / elapsedTime) * .0;//.007
        }

        double motorPower = p + slideI - d;
        slideTimer.reset();
        prevSlideError = error;

        hardwareQueue.add(() -> horizontalLeftMotor.setPower(motorPower));
        hardwareQueue.add(() -> horizontalRightMotor.setPower(motorPower));

//        if (gamepad1.right_bumper) {
//            holdingSample = true;
//        } else if (gamepad1.left_bumper) {
//            holdingSample = false;
//        }

        switch (intakeState) {
            case EXTENDING:
                //if slides past bar or need to be dropped
                //add more logic
                if (slidePos>10) {
                    intakePos = IntakePos.DOWN;
                    intakeTimer.reset();

                    intakeState = IntakeState.DROP_INTAKE;
                }
                break;
            case DROP_INTAKE:
                //if intake has finished dropping start intaking
                if (intakeTimer.seconds()>.5) {
                    targetIntakeSpeed = 1;

                    intakeState = IntakeState.INTAKING;
                }

            case INTAKING:
                if (holdingSample()) {
                    targetIntakeSpeed = .1;

                    intakePos = IntakePos.UP;

                    targetSlidePos = HorizontalSlide.IN.length;

                    intakeState = IntakeState.RETRACTING;
                }
                break;
            case RETRACTING:
                //if fully retracted and holding sample start transferring else rest
                if (slidePos<.3) {
                    if (holdingSample()) {
                        targetIntakeSpeed = -1;
                        intakeTimer.reset();

                        intakeState = IntakeState.TRANSFERING;
                    } else {
                        targetIntakeSpeed = 0;
                        intakePos = IntakePos.PARTIAL_UP;

                        intakeState = IntakeState.RESTING;
                    }
                }
                break;
            case TRANSFERING:
                //if transferred go to resting position, add more logic later
                if (intakeTimer.seconds()>.8) {
                    targetIntakeSpeed = 0;
                    intakePos = IntakePos.PARTIAL_UP;

                    intakeState = IntakeState.RESTING;
                }
                break;
        }
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {

        slidePosTelem.setValue(String.format("%.2f Target: %.2f", slidePos, targetSlidePos));

        return super.dashboard(packet);
    }

    private boolean holdingSample() {
        return holdingSample;
    }

    private double ticksToInches(int ticks) {
        return (ticks/145.1)*4.72;
    }

    public void setTargetSlidePos(HorizontalSlide targetPos) {
        setTargetSlidePos(targetPos.length);
    }

    public void setTargetSlidePos(double targetPos) {
        changedTargetSlidePos = true;
        newTargetSlidePos = MathUtil.clip(targetPos, 0, 18.7);//18.9 is max .2 to be safe
    }

    public void setIntakePos(IntakePos intakePos) {
        changedIntakePos = true;
        newIntakePos = intakePos;
    }

    public double getTargetSlidePos() {
        return prevTargetSlidePos;
    }

    public void setTargetIntakeSpeed(double intakeSpeed) {
        changedIntakeSpeed = true;
        newIntakeSpeed = intakeSpeed;
    }

    //changing intake states
    public void extendAndIntake(HorizontalSlide targetPos) {
        extendAndIntake(targetPos.length);
    }
    public void extendAndIntake(double targetPos) {
        setTargetSlidePos(targetPos);

        setIntakeState(IntakeState.EXTENDING);
    }

    public void retract() {
        setTargetIntakeSpeed(.1);
        setIntakePos(IntakePos.UP);
        setTargetSlidePos(HorizontalSlide.IN);

        setIntakeState(IntakeState.RETRACTING);
    }

    public void setIntakeState(IntakeState intakeState) {
        newIntakeState = intakeState;

        changedIntakeState = true;
    }
}
