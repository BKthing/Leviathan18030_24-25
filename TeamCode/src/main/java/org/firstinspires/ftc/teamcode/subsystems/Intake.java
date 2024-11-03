package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

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
        RETRACTING_INTAKE,
        RETRACTING,
        TRANSFERING,
        MOVING_TO_REST,
        RESTING
    }

    private IntakeState intakeState = IntakeState.RESTING;

    private IntakeState newIntakeState = IntakeState.RESTING;

    private boolean changedIntakeState = false;

    private final ElapsedTimer intakeTimer = new ElapsedTimer();

    private final Telemetry.Item slidePosTelem;

    public enum HorizontalSlide {
        //18.9 max
        EXTRA_IN(-1.5),
        IN(0),
        AUTO_PRESET1(12),
        AUTO_PRESET2(4),
        CLOSE(7),
        MEDIUM(12),
        FAR(16);//17

        public final double length;
        HorizontalSlide(double length) {this.length = length;}
    }

    private boolean changedTargetSlidePos = false;
    private double targetSlidePos = 0;
    private double newTargetSlidePos;

    private double prevTargetSlidePos = 0;

    private double slidePos;//inches
    private double prevSlideError = 0;

//    private double slideI = 0;

    private int slideTicks = 0;

    private final Encoder horizontalSlideEncoder;

    private final DcMotorEx horizontalLeftMotor, horizontalRightMotor;

    public enum IntakePos {
        UP(.435),//.69
        PARTIAL_UP(.36),//.6
        CLEAR_BAR(.62),
        DOWN(.122);//.05

        public final double pos;
        IntakePos(double pos) {this.pos = pos;}
    }
    private double targetIntakePos = IntakePos.PARTIAL_UP.pos;
    private double newIntakePos = IntakePos.PARTIAL_UP.pos;

    private double actualIntakePos = IntakePos.PARTIAL_UP.pos;
    private boolean changedIntakePos = false;
//    boolean updateIntakePos = true;

    private boolean changedIntakeSpeed = false;
    private double targetIntakeSpeed = 0;
    private double actualIntakeSpeed = 0;
    private double newIntakeSpeed = 0;

    private boolean updateTransfered = false;
    private boolean transfered = false;


    private final Servo leftIntakeServo, rightIntakeServo;
    private final CRServo intakeServo;

    private final ElapsedTimer slideTimer = new ElapsedTimer();

    private boolean holdingSample = true;

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
        leftIntakeServo.scaleRange(.34, .965);
        rightIntakeServo.scaleRange(.34, .965);

        leftIntakeServo.setPosition(targetIntakePos+.006);
        rightIntakeServo.setPosition(targetIntakePos);


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
//            horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hardwareMap.get(DcMotorEx.class, "verticalRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hardwareMap.get(DcMotorEx.class, "verticalRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        if (changedIntakePos && targetIntakePos != newIntakePos) {
            targetIntakePos = newIntakePos;
//            updateIntakePos = true;
            changedIntakePos = false;
        }

        if (changedIntakeSpeed) {
            targetIntakeSpeed = newIntakeSpeed;
            changedIntakeSpeed = false;
        }

        if (updateTransfered) {
            transfered = true;
            updateTransfered = false;
        }

        prevTargetSlidePos = targetSlidePos;
    }

    //trying to add to things to hardware queue asap so their more time for it to be called
    @Override
    public void loop() {

        //intake code
        if (Math.abs(targetIntakePos-actualIntakePos)>.005) {
            hardwareQueue.add(() -> leftIntakeServo.setPosition(targetIntakePos+.006));
            hardwareQueue.add(() -> rightIntakeServo.setPosition(targetIntakePos));

            actualIntakePos = targetIntakePos;
//            updateIntakePos = false;
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

        double p, d = 0, f = 0;

//        slideI += error*elapsedTime;

        //Checks if error is in acceptable amounts
        if (absError<.1) {
            p = 0;
        } else if (absError>4) {
            //Slides set to max power
            p = Math.signum(error);
        } else {//if (error<4 but error>.1)
            p = error*.35;
            d = ((prevSlideError-error) / elapsedTime) * .008;//.007
            f=.15*Math.signum(error);
        }

//        p = 0;
//        f=.3*Math.signum(error);

        double motorPower = (p - d)*Math.abs(p - d) + f;
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
                    targetIntakePos = IntakePos.DOWN.pos;
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

                    targetIntakePos = IntakePos.UP.pos;

//                    targetSlidePos = HorizontalSlide.IN.length;

                    intakeState = IntakeState.RETRACTING_INTAKE;

                    intakeTimer.reset();
                }
                break;
            case RETRACTING_INTAKE:
                if (intakeTimer.seconds()>.2) {
                    targetSlidePos = HorizontalSlide.EXTRA_IN.length;

                    intakeState = IntakeState.RETRACTING;
                }
                break;
            case RETRACTING:
                //if fully retracted and holding sample start transferring else rest
                if (slidePos<.5 && intakeTimer.seconds()>.6) {
                    if (holdingSample()) {
                        targetIntakeSpeed = -1;
                        intakeTimer.reset();

                        targetSlidePos = HorizontalSlide.IN.length;

                        intakeState = IntakeState.TRANSFERING;
                    } else {
                        targetIntakeSpeed = 0;
                        targetIntakePos = IntakePos.PARTIAL_UP.pos;

                        intakeState = IntakeState.RESTING;
                    }
                }
                break;
            case TRANSFERING:
                //if transferred go to resting position, add more logic later
                if (intakeTimer.seconds()>.5) {
                    targetIntakeSpeed = .2;
                    targetIntakePos = IntakePos.PARTIAL_UP.pos;

                    intakeState = IntakeState.MOVING_TO_REST;

                    intakeTimer.reset();
                }
                break;
            case MOVING_TO_REST:
                if (intakeTimer.seconds()>.2) {
                    updateTransfered = true;

                    targetIntakeSpeed = 0;

                    intakeState = IntakeState.RESTING;
                }
                break;
        }
    }

    @SuppressLint("DefaultLocale")
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
        newTargetSlidePos = MathUtil.clip(targetPos, 0, 18.5);//18.5
    }

    public void setTargetIntakePos(IntakePos targetIntakePos) {
        setTargetIntakePos(targetIntakePos.pos);
    }

    public void setTargetIntakePos(double intakePos) {
        changedIntakePos = true;
        newIntakePos = MathUtil.clip(intakePos, .1, .46);
    }

    public double getTargetIntakePos() {
        return newIntakePos;
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
        setTargetIntakePos(IntakePos.UP);
//        setTargetSlidePos(HorizontalSlide.IN);

        setIntakeState(IntakeState.RETRACTING_INTAKE);

        intakeTimer.reset();
    }

    public void setIntakeState(IntakeState intakeState) {
        newIntakeState = intakeState;

        changedIntakeState = true;
    }

    public boolean transfered() {
        if (transfered) {
            transfered = false;
            return true;
        } else {
            return false;
        }
    }
}
