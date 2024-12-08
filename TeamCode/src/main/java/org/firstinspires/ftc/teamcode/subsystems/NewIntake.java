package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class NewIntake extends SubSystem {

    public enum IntakeState {
        EXTENDING,
        DROPPING_INTAKE,
        INTAKING,
        RETRACTING_INTAKE,
        RETRACTING,
        WAITING_AFTER_RETRACTING,
        WAITING_FOR_TRANSFER,
        TRANSFERING,
        RESTING
    }

    private IntakeState intakeState = IntakeState.RESTING;

    private IntakeState newIntakeState = intakeState;

    private boolean updateIntakeState = false;


    public enum IntakingState {
        START_INTAKING,
        INTAKING,
        FINISH_INTAKING,
        HOLDING_SAMPLE,
        MANUAL_EJECTING,
        START_EJECTING,
        FINISH_EJECTING,

        START_REINTAKING,
        FINISH_REINTAKING,

        START_UNJAMMING,
        UNJAMMING_SPIN_OUT,
        UNJAMMING_SPIN_IN,
        UNJAMMING_FINNISH_SPIN_IN,

        START_PARTIAL_GRAB,
        INTAKING_PARTIAL_GRAB,

        START_EJECTING_PARTIAL_GRAB,
        EJECTING_PARTIAL_GRAB,

        IDLE
    }

    private IntakingState intakingState = IntakingState.IDLE;

    private IntakingState newIntakingState = intakingState;

    private IntakingState prevIntakingState = intakingState;

    private boolean updateIntakingState = false;

    private final ElapsedTimer intakeTimer = new ElapsedTimer();

    private final ElapsedTimer intakingTimer = new ElapsedTimer();

    private final Telemetry.Item slidePosTelem;


    public enum SampleColor{
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    SampleColor sampleColor = SampleColor.NONE;

    public enum ToIntakeState {
        DROP_INTAKE,
        RAISE_INTAKE,
        PARTIAL_RAISE_INTAKE,
        RAISE_TO_AUTO_HEIGHT,
        RETRACT,
        IDLE
    }

    private ToIntakeState toIntakeState = ToIntakeState.IDLE;

    private ToIntakeState newToIntakeState = toIntakeState;

    private boolean changedToIntakeState = false;

    private final Boolean blueAlliance;


    public enum HorizontalSlide {
        //18.9 max
        EXTRA_IN(-1),
        IN(0),
        AUTO_PRESET1(13.5),
        AUTO_PRESET2(4),
        CLOSE(7),
        MEDIUM(12),
        FAR(16);//17

        public final double length;
        HorizontalSlide(double length) {this.length = length;}
    }

    private double targetSlidePos = 0;
    private double newTargetSlidePos;
    private boolean changedTargetSlidePos = false;

    private double slidePos;//inches
    private double prevSlideError = 0;


    private int slideTicks = 0;

    private final Encoder horizontalSlideEncoder;

    private final DcMotorEx horizontalLeftMotor, horizontalRightMotor;

    private double actualMotorPower = 0;

    public enum IntakePos {
        UP(.07),//.69
        AUTO_HEIGHT(.1),
        PARTIAL_UP(.13),
        DOWN(.165);//.05

        public final double pos;
        IntakePos(double pos) {this.pos = pos;}
    }
    private double targetIntakePos = IntakePos.UP.pos;
    private double actualIntakePos = -1;

    private double targetIntakeSpeed = 0;
    private double actualIntakeSpeed = 0;


    private final Servo leftIntakeServo, rightIntakeServo;
    private final CRServo leftSpinnerServo, rightSpinnerServo;

    private NormalizedColorSensor colorSensor;

    private TouchSensor breakBeam;

    private boolean isBreakBeam = false;
    private boolean prevIsBreakBeam = false;

    private final ElapsedTimer slideTimer = new ElapsedTimer();

    private boolean transfer = false;

    private final boolean teleOpControls;

    NormalizedRGBA colors;
    NormalizedRGBA newColors;

    private boolean checkColor = false;

    Gamepad oldGamePad2 = new Gamepad();

    Telemetry.Item intakeTelem;
    Telemetry.Item colorTelem;

    public NewIntake(SubSystemData data, Boolean blueAlliance, boolean teleOpControls, boolean init) {
        super(data);

        this.teleOpControls = teleOpControls;
        this.blueAlliance = blueAlliance;

        //Motors
        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
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

        if (init) {
            leftIntakeServo.setPosition(targetIntakePos);
            rightIntakeServo.setPosition(targetIntakePos);
        }

        leftSpinnerServo = hardwareMap.get(CRServo.class, "leftSpinnerServo");
        rightSpinnerServo = hardwareMap.get(CRServo.class, "rightSpinnerServo");

        leftSpinnerServo.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        intakeTelem = telemetry.addData("Intake state", intakeState.name());
        colorTelem = telemetry.addData("Color Telem", "");
        //initiating slide encoder
        slidePos = ticksToInches(horizontalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {

            horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        slidePosTelem = telemetry.addData("Slide position", "");


    }

    @Override
    public void priorityData() {
//        if (changedIntakeState) {
//            intakeState = newIntakeState;
//            changedIntakeState = false;
//        }
//
        slideTicks = horizontalSlideEncoder.getCurrentPosition();

        isBreakBeam = breakBeam.isPressed();

        if (newColors != null && checkColor) {
            colors = newColors;
        }

        if ((isBreakBeam && !prevIsBreakBeam) && !checkColor && intakingState == IntakingState.INTAKING) {
            if (blueAlliance != null) {
                colors = colorSensor.getNormalizedColors();
            }
            checkColor = true;
        }

        if (updateIntakeState) {
            intakeState = newIntakeState;

            updateIntakeState = false;
        }

        if (updateIntakingState) {
            intakingState = newIntakingState;

            updateIntakingState = false;
        }

        if (changedToIntakeState) {
            toIntakeState = newToIntakeState;
            changedToIntakeState = false;
        }

        if (changedTargetSlidePos) {
            targetSlidePos = newTargetSlidePos;
            changedTargetSlidePos = false;
        }

        prevIntakingState = intakingState;
    }

    //trying to add to things to hardware queue asap so their more time for it to be called
    @Override
    public void loop() {
        if (teleOpControls) {
            if (gamepad2.back) {
                if (gamepad2.dpad_down) {
                    horizontalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    targetSlidePos = 0;
                } else if (gamepad2.dpad_up) {
                    intakingState = IntakingState.START_UNJAMMING;
                }

                if (Math.abs(gamepad2.left_stick_y)>.05) {
                    targetSlidePos = targetSlidePos + 10 * slideTimer.seconds() * -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * .75);
                }
            } else {
                if (gamepad2.dpad_down && !oldGamePad2.dpad_down) {
                    toIntakeState = ToIntakeState.RETRACT;
                    if (intakingState != IntakingState.FINISH_INTAKING) {
                        intakingState = IntakingState.IDLE;
                        targetIntakeSpeed = 0;
                    }
                } else if (gamepad2.dpad_right && !oldGamePad2.dpad_right) {
                    targetSlidePos = HorizontalSlide.CLOSE.length;
                } else if (gamepad2.dpad_left && !oldGamePad2.dpad_left) {
                    targetSlidePos = HorizontalSlide.MEDIUM.length;
                } else if (gamepad2.dpad_up && !oldGamePad2.dpad_up) {
                    targetSlidePos = HorizontalSlide.FAR.length;
                } else if (Math.abs(gamepad2.left_stick_y) > .05) {
                    if (!gamepad2.start) {
                        targetSlidePos = MathUtil.clip(targetSlidePos + 10 * slideTimer.seconds() * -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * .75), -.5, 18.5);
    //                    intake.setTargetSlidePos(intake.getTargetSlidePos() + 10 * loopTimer.seconds() * -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * .75));
                    } else {
    //                    intake.setTargetIntakePos(intake.getTargetIntakePos() + gamepad2.left_stick_y * .0002 * loopTimer.milliSeconds());
                    }
            }
            }

            if (gamepad2.left_bumper && !oldGamePad2.left_bumper) {
                toIntakeState = ToIntakeState.DROP_INTAKE;
                intakingState = IntakingState.INTAKING;
                targetIntakeSpeed = .6;
            } else if (gamepad2.left_trigger>.2 && oldGamePad2.left_trigger<=.2) {
                targetIntakeSpeed = -1;
                intakingState = IntakingState.MANUAL_EJECTING;
            } else if ((!gamepad2.left_bumper && oldGamePad2.left_bumper) || (oldGamePad2.left_trigger>.2 && gamepad2.left_trigger<=.2)) {
                if (intakingState == IntakingState.INTAKING || intakingState == IntakingState.MANUAL_EJECTING) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                    checkColor = false;
                }
            }
        }

        switch (toIntakeState) {
            case DROP_INTAKE:
                targetIntakePos = IntakePos.DOWN.pos;

                intakeState = IntakeState.INTAKING;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case PARTIAL_RAISE_INTAKE:
                targetIntakePos = IntakePos.PARTIAL_UP.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RAISE_TO_AUTO_HEIGHT:
                targetIntakePos = IntakePos.AUTO_HEIGHT.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RAISE_INTAKE:
                targetIntakePos = IntakePos.UP.pos;

                toIntakeState = ToIntakeState.IDLE;
                break;
            case RETRACT:
                targetIntakePos = IntakePos.UP.pos;
                intakeTimer.reset();

                intakeState = IntakeState.RETRACTING_INTAKE;

                toIntakeState = ToIntakeState.IDLE;
                break;
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
            d = ((prevSlideError-error) / elapsedTime) * .01;//.007
            f=.15*Math.signum(error);
        }

//        p = 0;
//        f=.3*Math.signum(error);

        double motorPower = (p - d)*Math.abs(p - d) + f;
        slideTimer.reset();
        prevSlideError = error;


        if (Math.abs(motorPower-actualMotorPower)>.04) {
            hardwareQueue.add(() -> horizontalLeftMotor.setPower(motorPower));
            hardwareQueue.add(() -> horizontalRightMotor.setPower(motorPower));

            actualMotorPower = motorPower;
        }



        if (Math.abs(targetIntakePos-actualIntakePos)>.01) {
            hardwareQueue.add(() -> leftIntakeServo.setPosition(targetIntakePos));
            hardwareQueue.add(() -> rightIntakeServo.setPosition(targetIntakePos));

            actualIntakePos = targetIntakePos;
        }

        if (Math.abs(targetIntakeSpeed-actualIntakeSpeed)>.04) {
            hardwareQueue.add(() -> leftSpinnerServo.setPower(targetIntakeSpeed));
            hardwareQueue.add(() -> rightSpinnerServo.setPower(targetIntakeSpeed));

            actualIntakeSpeed = targetIntakeSpeed;
        }

        switch (intakingState) {
            case START_INTAKING:
                targetIntakeSpeed = .6;
                intakingState = IntakingState.INTAKING;
                break;
            case INTAKING:
                if (checkColor) {
                    if (blueAlliance == null) {
                        targetIntakePos = IntakePos.UP.pos;

                        intakeState = IntakeState.RETRACTING_INTAKE;

                        intakingState = IntakingState.FINISH_INTAKING;

                        intakingTimer.reset();
                        intakeTimer.reset();

                        checkColor = false;

                        break;
                    }

                    sampleColor = findSampleColor();

                    if (sampleColor == SampleColor.NONE) {
                        hardwareQueue.add(() -> {
                            newColors = colorSensor.getNormalizedColors();
                        });
                        break;
                    }

                    checkColor = false;

                    if ((sampleColor == SampleColor.BLUE && !blueAlliance) ||  (sampleColor == SampleColor.RED && blueAlliance)) {
                        targetIntakeSpeed = -1;
                        targetIntakePos = IntakePos.PARTIAL_UP.pos;
                        intakingState = IntakingState.START_EJECTING;
                        intakingTimer.reset();
                    } else {
//                            targetIntakeSpeed = 1;
                        targetIntakePos = IntakePos.UP.pos;

                        intakeState = IntakeState.RETRACTING_INTAKE;

                        intakingState = IntakingState.FINISH_INTAKING;

                        intakingTimer.reset();
                        intakeTimer.reset();
                    }

                }
                break;
            case FINISH_INTAKING:
                if (intakingTimer.seconds()>1 || targetIntakeSpeed == 0) {
                    targetIntakeSpeed = 0;

                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;
            case START_EJECTING:
                if (!isBreakBeam || intakingTimer.seconds()>1) {
                    intakingTimer.reset();

                    intakingState = IntakingState.FINISH_EJECTING;
                }
                break;
            case FINISH_EJECTING:
                if (intakingTimer.seconds()>1) {
                    targetIntakeSpeed = 0;
                    targetIntakePos = IntakePos.DOWN.pos;
                    intakingState = IntakingState.IDLE;
                }
                break;
            case MANUAL_EJECTING:

                break;


            case START_REINTAKING:
                targetIntakeSpeed = 1;
                intakingTimer.reset();

                intakingState = IntakingState.FINISH_INTAKING;
                break;
            case FINISH_REINTAKING:
                if (intakingTimer.seconds()>1.5) {
                    targetIntakeSpeed = 0;

                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;


            case START_UNJAMMING:
                intakingTimer.reset();
                targetIntakeSpeed = -1;
                intakingState = IntakingState.UNJAMMING_SPIN_OUT;
                break;
            case UNJAMMING_SPIN_OUT:
                if (intakingTimer.seconds()>.25) {
                    targetIntakeSpeed = 1;
                    intakingState = IntakingState.UNJAMMING_SPIN_IN;
                    intakingTimer.reset();
                }
                break;
            case UNJAMMING_SPIN_IN:
                if (isBreakBeam || intakingTimer.seconds() > .5) {
                    intakingTimer.reset();
                    intakingState = IntakingState.UNJAMMING_FINNISH_SPIN_IN;
                }
                break;
            case UNJAMMING_FINNISH_SPIN_IN:
                if (intakingTimer.seconds()>1.5) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.HOLDING_SAMPLE;
                }
                break;


            case START_PARTIAL_GRAB:
                targetIntakeSpeed = .3;
                intakingState = IntakingState.INTAKING_PARTIAL_GRAB;
                intakingTimer.reset();
                break;
            case INTAKING_PARTIAL_GRAB:
                if (isBreakBeam || intakingTimer.seconds()>1) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                }
                break;

            case START_EJECTING_PARTIAL_GRAB:
                targetIntakeSpeed = -1;
                targetIntakePos = IntakePos.PARTIAL_UP.pos;
                intakingState = IntakingState.INTAKING_PARTIAL_GRAB;
                intakingTimer.reset();
                break;
            case EJECTING_PARTIAL_GRAB:
                if (intakingTimer.seconds()>.3) {
                    targetIntakeSpeed = 0;
                    intakingState = IntakingState.IDLE;
                }
                break;
        }


        switch (intakeState) {
            case EXTENDING:
                //if slides past bar or need to be dropped
                //add more logic
                if (slidePos>10) {
                    targetIntakePos = IntakePos.DOWN.pos;
                    intakeTimer.reset();

                    intakeState = IntakeState.DROPPING_INTAKE;
                }
                break;
            case DROPPING_INTAKE:
                //if intake has finished dropping start intaking
                if (intakeTimer.seconds()>.2) {
                    targetIntakeSpeed = .6;



                    intakeState = IntakeState.INTAKING;
                }

            case INTAKING:
//                if (holdingSample) {
//                    targetIntakeSpeed = .1;
//
//                    targetIntakePos = IntakePos.UP.pos;
//
////                    targetSlidePos = HorizontalSlide.IN.length;
//
//                    intakeState = IntakeState.RETRACTING_INTAKE;
//
//                    intakeTimer.reset();
//                }
                break;
            case RETRACTING_INTAKE:
                if (intakeTimer.seconds()>.3) {
                    targetSlidePos = HorizontalSlide.EXTRA_IN.length;

                    intakeState = IntakeState.RETRACTING;
                }
                break;
            case RETRACTING:
                //if fully retracted and holding sample start transferring else rest
                if (slidePos<.4 || intakeTimer.seconds()>1) {
                    intakeTimer.reset();

                    intakeState = IntakeState.WAITING_AFTER_RETRACTING;
                }
                break;
            case WAITING_AFTER_RETRACTING:
                if (intakeTimer.seconds()>.3) {
                    targetSlidePos = HorizontalSlide.IN.length;

                    if (intakingState == IntakingState.HOLDING_SAMPLE || (isBreakBeam && intakingState == IntakingState.IDLE)) {
                        targetIntakeSpeed = 0;
                        if (isBreakBeam) {
                            transfer = true;
                            intakeState = IntakeState.TRANSFERING;
                        } else {
                            intakeState = IntakeState.WAITING_FOR_TRANSFER;
                            intakingState = IntakingState.START_UNJAMMING;
                        }

                    } else {
                        intakeState = IntakeState.WAITING_FOR_TRANSFER;
                    }
                }
                break;
            case WAITING_FOR_TRANSFER:
                if (intakingState == IntakingState.HOLDING_SAMPLE) {
                    if (isBreakBeam) {
                        transfer = true;
                        intakeState = IntakeState.TRANSFERING;
                    } else {
                        intakingState = IntakingState.START_UNJAMMING;
                    }
                }
                break;
            case TRANSFERING:
                //if transferred go to resting position, add more logic later
                if (!transfer) {
                    intakeState = IntakeState.RESTING;
                    intakingState = IntakingState.IDLE;
                }
                break;
        }

        intakeTelem.setValue(intakeState.name());
        colorTelem.setValue(checkColor + " cur:" + isBreakBeam + " prev:" + prevIsBreakBeam);

        prevIsBreakBeam = isBreakBeam;
        oldGamePad2.copy(gamepad2);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {

        slidePosTelem.setValue(String.format("%.2f Target: %.2f", slidePos, targetSlidePos));

        return super.dashboard(packet);
    }

    private SampleColor findSampleColor() {
        if (colors.red > .01 && colors.green < .012) {
//            throw new RuntimeException("Not red");
            return SampleColor.RED;
        }
        else if (colors.blue > .01) {
            return SampleColor.BLUE;
        }
        else if (colors.green > .015) {
//            throw new RuntimeException("Not yellow");
            return SampleColor.YELLOW;
        }
        else {
//            throw new RuntimeException("Not nothing");
            return SampleColor.NONE;
        }
    }

    public boolean holdingSample() {
        return breakBeam.isPressed();
    }

    private double ticksToInches(int ticks) {
        return (ticks/145.1)*4.72;
    }


    //only call during priority data to avoid threading issues
    public boolean transfer() {
        if (transfer) {
            transfer = false;
            return true;
        } else {
            return false;
        }
    }

    public SampleColor getSampleColor() {
        return sampleColor;
    }


    //single thread safe
    public void setIntakingState(IntakingState intakingState) {
        newIntakingState = intakingState;
        updateIntakingState = true;
    }

    public void setIntakeState(IntakeState intakeState) {
        newIntakeState = intakeState;
        updateIntakeState = true;
    }

    public void toIntakeState(ToIntakeState toIntakeState) {
        newToIntakeState = toIntakeState;
        changedToIntakeState = true;
    }

    public void setTargetSlidePos(double distance) {
        newTargetSlidePos = distance;
        changedTargetSlidePos = true;
    }

    public IntakingState getPrevIntakingState() {
        return prevIntakingState;
    }

}
