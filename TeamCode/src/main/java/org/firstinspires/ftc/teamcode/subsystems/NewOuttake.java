package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Range;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class NewOuttake extends SubSystem {

    public enum OuttakeState {
        EXTENDING_PLACE_BEHIND,
        WAITING_PLACE_BEHIND,
        PLACING_BEHIND,
        RETRACTING_FROM_PLACE_BEHIND,
        WAITING_FOR_TRANSFER,
        MOVING_TO_TRANSFER,
        GRABBING_FROM_TRANSFER,
        EXTRACTING_FROM_TRANSFER,

        WAITING_GRAB_SPECIMEN,
        GRABBING_SPECIMEN,
        REMOVING_SPECIMEN_FROM_WALL,
        EXTENDING_CLEAR_TRANSFER,
        EXTENDING_PLACE_FRONT,
        WAITING_PLACE_FRONT,
        PLACING_FRONT,
        MOVING_TO_CLEAR_BAR,
        RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE,
        RETRACTING_TO_GRAB_SPECIMEN,

        INIT_POSITION,
        IDLE
    }

    private OuttakeState outtakeState = OuttakeState.INIT_POSITION;

    public enum ToOuttakeState {
        WAIT_PLACE_FRONT,
        PLACE_FRONT,
        WAIT_FOR_TRANSFER,
        WAIT_PLACE_BEHIND,
        PLACE_BEHIND,
        INIT_POSITION,
        IDLE
    }

    private ToOuttakeState toOuttakeState = ToOuttakeState.INIT_POSITION;

    private ToOuttakeState newToOuttakeState = toOuttakeState;

    private boolean changedToOuttakeState = true;


    private final ElapsedTimer outtakeTimer = new ElapsedTimer();


    public enum VerticalSlide {
        EXTRA_DOWN(-.3),
        DOWN(0),
        WAIT_FOR_TRANSFER(5),
        TRANSFER(-.5),
        EXTRACT_FROM_TRANSFER(1),
        MIN_PASSTHROUGH_HEIGHT(8.5),
        SPECIMEN_PICKUP(6),
        SPECIMEN_BAR(8),
        PLACE_SPECIMEN_BAR(4),
        HANG_HEIGHT(17.5),
        LOW_BUCKET_HEIGHT(20),
        HIGH_BUCKET(25);

        public final double length;
        VerticalSlide(double length) {this.length = length;}
    }

    private final ElapsedTimer slideTimer = new ElapsedTimer();

    private double targetSlidePos, newTargetSlidePos;

    private double slidePos;

    private double prevSlideError;

    private int slideTicks = 0;

    public enum V4BarPos {
        PLACE_FRONT(.37),
        WAIT_FOR_TRANSFER(.15),
        TRANSFER(.128),
        EXTRACT_FROM_TRANSFER(.12),
        GRAB_BACK(.0),
        EXTRACT_FROM_GRAB_BACK(.1),
        PLACE_BACK(.8),
        IDLE_POSITION(.5);

        public final double pos;

        V4BarPos(double pos) {
            this.pos = pos;
        }
    }

    private double targetV4BPos = V4BarPos.IDLE_POSITION.pos;

    private double actualV4BPos = targetV4BPos;//set to -1 so target will never == actual on first loop


    public enum ClawPitch {
        DOWN(.21),
        BACK(0.35),
        WAIT_FOR_TRANSFER(.49),
        TRANSFER(.49),
        EXTRACT_FROM_TRANSFER(.49),
        FRONT_ANGLED_UP(.84),
        FRONT_ANGELED_DOWN(.59),

        FRONT(.3);

        public final double pos;

        ClawPitch(double pos) {
            this.pos = pos;
        }
    }

    private double targetClawPitch = ClawPitch.DOWN.pos;

    private double actualClawPitch = targetClawPitch;


    public enum ClawPosition {
        EXTRA_OPEN(.6),
        OPEN(.4),
        CLOSED(.07);

        public final double pos;

        ClawPosition(double pos) {
            this.pos = pos;
        }
    }

    private ClawPosition clawPosition = ClawPosition.CLOSED;

    private boolean updateClawPosition = false;


    public enum HangDeployPosition {
        DELPOYED(.5),
        RETRACTED(.8);

        public final double pos;

        HangDeployPosition(double pos) {
            this.pos = pos;
        }
    }

    private boolean transfer = false;

    private boolean updateTransfer = false;

    private final boolean teleOpControls;
    private final boolean autoExtendSlides, autoRetractSlides;


    private final DcMotorEx verticalLeftMotor,verticalRightMotor;
    private final Encoder verticalSlideEncoder;

    private Gamepad oldGamePad2 = new Gamepad();


    private final Servo clawServo, clawPitchServo, leftOuttakeServo, rightOuttakeServo;

    public NewOuttake(SubSystemData data, boolean teleOpControls, boolean autoExtendSlides, boolean autoRetractSlides) {
        super(data);

        this.teleOpControls = teleOpControls;
        this.autoExtendSlides = autoExtendSlides;
        this.autoRetractSlides = autoRetractSlides;


        //motors
        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft"); // control hub 2
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight"); // control hub 3

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //encoder
        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));
//        verticalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        //servos
        clawServo = hardwareMap.get(Servo.class, "clawServo"); // ex hub 4

        clawPitchServo = hardwareMap.get(Servo.class, "clawPitchServo"); // control hub 1

        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo"); // ex hub 5
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo"); // control hub 4

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);


        //initiating slide encoder
        slidePos = ticksToInches(verticalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            verticalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//            verticalRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        leftOuttakeServo.setPosition(targetV4BPos);
        rightOuttakeServo.setPosition(targetV4BPos);

        clawPitchServo.setPosition(targetClawPitch);

        clawServo.setPosition(clawPosition.pos);


    }


    @Override
    public void priorityData() {
        slideTicks = verticalSlideEncoder.getCurrentPosition();

        if (updateTransfer) {
            transfer = true;
            updateTransfer = false;
        }
    }

    @Override
    public void loop() {
        if (teleOpControls) {
            if (gamepad2.right_trigger>.2 && oldGamePad2.right_trigger<=.2) {
//                outtake.grabFromTransfer();
            } else if (gamepad2.a && !oldGamePad2.a) {
                targetSlidePos = VerticalSlide.TRANSFER.length;
//                outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
            } else if (gamepad2.b && !oldGamePad2.b) {
                targetSlidePos = VerticalSlide.SPECIMEN_BAR.length;
//                outtake.setTargetSlidePos(15);

//            outtake.toOuttakeState(Outtake.ToOuttakeState.GRAB_BEHIND);
            } else if (gamepad2.x && !oldGamePad2.x) {
                targetSlidePos = VerticalSlide.LOW_BUCKET_HEIGHT.length;
//                outtake.setTargetSlidePos(Outtake.VerticalSlide.LOW_BUCKET_HEIGHT);

//            outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_FRONT);
            } else if (gamepad2.y && !oldGamePad2.y) {
                targetSlidePos = VerticalSlide.HIGH_BUCKET.length;
//                outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_BEHIND);
            } else if (Math.abs(gamepad2.right_stick_y) > .05) {
                if (!gamepad2.back) {
                    targetSlidePos = MathUtil.clip(targetSlidePos+8 * slideTimer.seconds() * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75), -.5, 28.1);
                } else {
//                    outtake.setTargetV4BarPos(outtake.getTargetV4BarPos()+gamepad2.right_stick_y * .0002 * loopTimer.milliSeconds());
                }
            }
        }


        //slide PID
        slidePos = ticksToInches(slideTicks);
        //limits max time
        double elapsedTime = Math.min(slideTimer.seconds(), .5);

        //pid control
        double error = targetSlidePos - slidePos;
        double absError = Math.abs(error);

        double p, d = 0;

        double f;

        if (targetSlidePos != 0) {
            f = .13;
        } else {
            f = 0;
        }

        //Checks if error is in acceptable amounts

        if (absError>1) {
            //Slides set to max power
            p = Math.signum(error);
        } else {
            p =error*.38;
            d = ((prevSlideError-error) / elapsedTime) * .015;
        }

        double motorPower =  p  - d + f; //
        slideTimer.reset();
        prevSlideError = error;

        hardwareQueue.add(() -> verticalRightMotor.setPower(motorPower));
        hardwareQueue.add(() -> verticalLeftMotor.setPower(motorPower));

        switch (outtakeState) {
            case EXTENDING_PLACE_BEHIND:
                if (absError<.5) {
                    outtakeState = OuttakeState.WAITING_PLACE_BEHIND;
                }
                break;
            case WAITING_PLACE_BEHIND:
                //claw opened by other code calling method
                if (clawPosition == ClawPosition.OPEN && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PLACING_BEHIND;
                }
                break;
            case PLACING_BEHIND:
                if (outtakeTimer.seconds()>.3) {
                    retractFromPlaceBehind();
                }
                break;
            case RETRACTING_FROM_PLACE_BEHIND:
                if (outtakeTimer.seconds()>1.4) {
                    targetSlidePos = VerticalSlide.WAIT_FOR_TRANSFER.length;

                    outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                }
                break;
            case WAITING_FOR_TRANSFER:
                if (transfer) {
                    targetSlidePos = VerticalSlide.TRANSFER.length;
                    targetV4BPos = V4BarPos.TRANSFER.pos;
                    targetClawPitch = ClawPitch.TRANSFER.pos;

                    outtakeTimer.reset();
                    transfer = false;

                    outtakeState = OuttakeState.MOVING_TO_TRANSFER;
                }
                break;
            case MOVING_TO_TRANSFER:
                if (outtakeTimer.seconds()>.2) {
                    clawPosition = ClawPosition.CLOSED;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.GRABBING_FROM_TRANSFER;
                }
                break;
            case GRABBING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.2) {
                    targetSlidePos = VerticalSlide.EXTRACT_FROM_TRANSFER.length;
                    targetV4BPos = V4BarPos.EXTRACT_FROM_TRANSFER.pos;
                    targetClawPitch = ClawPitch.EXTRACT_FROM_TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EXTRACTING_FROM_TRANSFER;
                }
                break;
            case EXTRACTING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.2) {
                    if (autoExtendSlides) {
                        extendPlaceBehind();
                    } else {
                        outtakeState = OuttakeState.IDLE;
                    }
                }
                break;



            case WAITING_GRAB_SPECIMEN:
                if (clawPosition == ClawPosition.CLOSED && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.GRABBING_SPECIMEN;
                }
                break;
            case GRABBING_SPECIMEN:
                if (outtakeTimer.seconds()>.2) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length+1;
                    targetV4BPos = V4BarPos.EXTRACT_FROM_GRAB_BACK.pos;
                    targetClawPitch = ClawPitch.EXTRACT_FROM_TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.REMOVING_SPECIMEN_FROM_WALL;
                }
                break;

            case REMOVING_SPECIMEN_FROM_WALL:
                if (outtakeTimer.seconds()>.3) {
                    targetSlidePos = VerticalSlide.SPECIMEN_BAR.length;

                    outtakeState = OuttakeState.EXTENDING_CLEAR_TRANSFER;
                }
                break;
            case EXTENDING_CLEAR_TRANSFER:
                if (slidePos>VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length) {
                    targetV4BPos = V4BarPos.PLACE_BACK.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
                }
                break;
            case EXTENDING_PLACE_FRONT:
                if (outtakeTimer.seconds()>.6) {
                    outtakeState = OuttakeState.WAITING_PLACE_FRONT;
                }
                break;
            case WAITING_PLACE_FRONT:
                if (clawPosition == ClawPosition.OPEN) {
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.PLACING_FRONT;
                }
                break;
            case PLACING_FRONT:
                if (outtakeTimer.seconds()>.2) {

                }
                break;


//                    PLACING_FRONT,
//                    MOVING_TO_CLEAR_BAR,
//                    RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE,
//                    RETRACTING_TO_GRAB_SPECIMEN,

        }

        oldGamePad2.copy(gamepad2);

    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        return packet;
    }

    private double ticksToInches(int ticks) {
        return (ticks/384.5)*4.72;
    }

    private void extendPlaceBehind() {
        targetV4BPos = V4BarPos.PLACE_BACK.pos;
        targetClawPitch = ClawPitch.BACK.pos;

        targetSlidePos = VerticalSlide.HIGH_BUCKET.length;

        outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
    }

    private void extendPlaceFront() {
        targetV4BPos = V4BarPos.PLACE_FRONT.pos;
        targetClawPitch = ClawPitch.FRONT.pos;

        targetSlidePos = VerticalSlide.SPECIMEN_BAR.length;

        outtakeState = OuttakeState.PLACING_FRONT;
    }

//    private void extend

    private void grabBehind() {
        targetV4BPos = V4BarPos.GRAB_BACK.pos;
        targetClawPitch = ClawPitch.BACK.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        //set to this so V4B has room to rotate, set lower after v4b is clear
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACTING_TO_GRAB_SPECIMEN;
    }

    private void retractFromPlaceBehind() {
        targetV4BPos = V4BarPos.WAIT_FOR_TRANSFER.pos;
        targetClawPitch = ClawPitch.BACK.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        //set to this so V4B has room to rotate, set lower after v4b is clear

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACTING_FROM_PLACE_BEHIND;
    }

    public void transfer() {
        updateTransfer = true;
    }
}
