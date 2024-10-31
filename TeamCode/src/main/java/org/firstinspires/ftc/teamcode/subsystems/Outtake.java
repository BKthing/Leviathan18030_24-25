package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class Outtake extends SubSystem {

    public enum OuttakeState {
        EXTENDING_PLACE_BEHIND,
        PLACING_BEHIND,
        PLACING_BEHIND_RETRACT_DELAY,
        RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B,
        RETRACT_FROM_PLACE_BEHIND_DELAY,
        RETRACT_FROM_PLACE_BEHIND,
        EXTENDING_PLACE_FRONT,
        PLACING_FRONT,
        WAIT_PLACING_FRONT,
        PLACING_FRONT_RETRACT_DELAY,
        RETRACTING_GRAB_BEHIND_CLEAR_V4B,
        RETRACTING_GRAB_BEHIND,
        WAITING_GRAB_BEHIND,
        GRABBING_BEHIND_EXTEND_DELAY,
        LIFT_SPECIMEN,
        WAITING_FOR_TRANSFER,
        PICK_UP_FROM_TRANSFER,
        WAIT_FOR_PICK_UP_FROM_TRANSFER,
        IDLE,

        INIT_POSITION
    }

    private OuttakeState outtakeState = OuttakeState.INIT_POSITION;
    private OuttakeState newOuttakeState = OuttakeState.INIT_POSITION;

    private boolean changedOuttakeState = false;

    public enum ToOuttakeState {
        EXTEND_PLACE_BEHIND,
        EXTEND_PLACE_FRONT,
        GRAB_BEHIND,
        RETRACT_FROM_PLACE_BEHIND,
        RETRACT,
        NOTHING
    }

    private ToOuttakeState toOuttakeState = ToOuttakeState.NOTHING;
    private ToOuttakeState targetToOuttakeState = ToOuttakeState.NOTHING;

    private boolean updateToOuttakeState = false;



    private final ElapsedTimer outtakeTimer = new ElapsedTimer();


    public enum VerticalSlide {
        //28.35in max
        BOTTOM(0),
        WAIT_FOR_TRANSFER(5),
        TRANSFER(-.5),
        MIN_PASSTHROUGH_HEIGHT(8.5),
        SPECIMEN_PICKUP(6),
        SPECIMEN_BAR(16),
        PLACE_SPECIMEN_BAR(15),
        LOW_BUCKET_HEIGHT(20),
        HIGH_BUCKET(28.5);

        public final double length;
        VerticalSlide(double length) {this.length = length;}
    }


    private final Telemetry.Item outtakeSlideTelemetry;

    private final ElapsedTimer slideTimer = new ElapsedTimer();

    private boolean changedTargetSlidePos = false;

    private double targetSlidePos;
    private double newTargetSlidePos;
    private double prevTargetSlidePos;

    private double slidePos;//inches
    private double prevSlideError = 0;

    private double slideI = 0;

    private int slideTicks = 0;

    private final Encoder verticalSlideEncoder;

    private final DcMotorEx verticalLeftMotor, verticalRightMotor;


    public enum V4BarPos {
        PLACE_FRONT(.37),
        WAIT_FOR_TRANSFER(.1645),
        TRANSFER(.166),
        GRAB_BACK(.0),
        PLACE_BACK(.89);

        public final double pos;

        V4BarPos(double pos) {
            this.pos = pos;
        }
    }
    private boolean changedV4BarPos = false;
    private double targetV4BarPos = V4BarPos.PLACE_FRONT.pos;
    private double actualV4BarPos = V4BarPos.PLACE_FRONT.pos;
    private double newV4BarPos = V4BarPos.PLACE_FRONT.pos;

    private final Servo leftOuttakeServo, rightOuttakeServo;


    public enum WristPitch {
        DOWN(.54),
        BACK(0.24),
        WAIT_FOR_TRANSFER(.49),
        Front_ANGLED(.9),
        FRONT(.81);

        public final double pos;

        WristPitch(double pos) {
            this.pos = pos;
        }
    }
    private boolean changedWristPitch = false;
    private double targetWristPitch = WristPitch.BACK.pos;
    private double actualWristPitch = WristPitch.BACK.pos;
    private double newWristPitch = WristPitch.BACK.pos;

    private final Servo wristPitchServo;


    public enum WristRoll {
        NEGATIVE_NINETY(.34),
        ZERO(.249),
        NINETY(.145);

        public final double pos;

        WristRoll(double pos) {
            this.pos = pos;
        }
    }

    private boolean changedWristRoll = false;
    private double targetWristRoll = WristRoll.NINETY.pos;
    private double actualWristRoll = WristRoll.NINETY.pos;
    private double newWristRoll = WristRoll.NINETY.pos;

    private final Servo wristRollServo;


    public enum ClawPosition {
        OPEN(.35),
        CLOSED(.1);

        public final double pos;

        ClawPosition(double pos) {
            this.pos = pos;
        }
    }

    private ClawPosition clawPosition = ClawPosition.CLOSED;
    private ClawPosition newClawPosition = ClawPosition.CLOSED;

    private boolean updateClawPosition = false;

    private boolean changedClawPosition = false;

    private boolean changedHangDeploy = false;

    private boolean updatedHangPos = false;

    private final Servo clawServo;

    private boolean grabFromTransfer = false;
    private boolean changedGrabFromTransfer = false;


    private final boolean autoExtendSlides;
    private final boolean autoRetractSlides;
//    boolean toggleAutoExtendSlides = false;


    private final Servo hangDeploy;

    private boolean place = false;
    private boolean changedPlace = false;

    private final Telemetry.Item V4BTelemetry;

    public enum HangDeploy {
        DEPLOY(.4),
        NOTDEPLOYED(.2);

        public final double pos;

        HangDeploy(double pos) {this.pos = pos;}
    }

    private HangDeploy hangDeployPos = HangDeploy.NOTDEPLOYED;
    private HangDeploy newHangDeploy = HangDeploy.NOTDEPLOYED;


    public Outtake(SubSystemData data, boolean autoExtendSlides, boolean autoRetractSlides) {
        super(data);

        this.autoExtendSlides = autoExtendSlides;
        this.autoRetractSlides = autoRetractSlides;

        //Motors
        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        verticalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft"); // control hub 2
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight"); // control hub 3

//        verticalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servos
        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo"); // ex hub 5
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo"); // control hub 4

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);

        leftOuttakeServo.scaleRange(.34, .965);
        rightOuttakeServo.scaleRange(1-.965, 1-.34);

        leftOuttakeServo.setPosition(actualV4BarPos);
        rightOuttakeServo.setPosition(actualV4BarPos);

        wristPitchServo = hardwareMap.get(Servo.class, "wristPitchServo"); // control hub 1
        wristRollServo = hardwareMap.get(Servo.class, "wristRollServo"); // control hub 3

        wristPitchServo.scaleRange(.34, .965);
        wristRollServo.scaleRange(.34, .965);

        wristPitchServo.setPosition(actualWristPitch);
        wristRollServo.setPosition(actualWristRoll);

        clawServo = hardwareMap.get(Servo.class, "clawServo"); // ex hub 4

        clawServo.setPosition(clawPosition.pos);

        hangDeploy = hardwareMap.get(Servo.class, "hangDeploy");


        //initiating slide encoder
        slidePos = ticksToInches(verticalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            hardwareMap.get(DcMotorEx.class, "horizontalLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//            verticalRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;

        V4BTelemetry = telemetry.addData("V4B pos", 0);

        outtakeSlideTelemetry = telemetry.addData("Outtake target pos", "");
    }


    @Override
    public void priorityData() {
        if (changedOuttakeState) {
            outtakeState = newOuttakeState;
            changedOuttakeState = false;
        }

        slideTicks = verticalSlideEncoder.getCurrentPosition();

        if(changedTargetSlidePos) {
            targetSlidePos = newTargetSlidePos;
            changedTargetSlidePos = false;
        }

        if (changedV4BarPos) {
            targetV4BarPos = newV4BarPos;
            changedV4BarPos = false;
        }

        if (changedWristPitch) {
            targetWristPitch = newWristPitch;
            changedWristPitch = false;
        }

        if (changedWristRoll) {
            targetWristRoll = newWristRoll;
            changedWristRoll = false;
        }

        if (changedClawPosition) {
            clawPosition = newClawPosition;
            updateClawPosition = true;
            changedClawPosition = false;
        }

        if (changedGrabFromTransfer) {
            grabFromTransfer = true;
            changedGrabFromTransfer = false;
        }

        if (changedHangDeploy) {
            hangDeployPos = newHangDeploy;
            updatedHangPos = true;
            changedHangDeploy = false;
        }

        if (changedPlace) {
            place = true;
            changedPlace = false;
        }


//        if (toggleAutoExtendSlides) {
//            autoExtendSlides = !autoExtendSlides;
//            toggleAutoExtendSlides = false;
//        }

        if (updateToOuttakeState) {
            toOuttakeState = targetToOuttakeState;
            updateToOuttakeState = false;
        }

        prevTargetSlidePos = targetSlidePos;
    }

    @Override
    public void loop() {
        switch (toOuttakeState) {
            case RETRACT_FROM_PLACE_BEHIND:
                retractFromPlaceBehind();
                toOuttakeState = ToOuttakeState.NOTHING;
                break;
            case GRAB_BEHIND:
                grabBehind();
                toOuttakeState = ToOuttakeState.NOTHING;
                break;
            case EXTEND_PLACE_FRONT:
                extendPlaceFront();
                toOuttakeState = ToOuttakeState.NOTHING;
                break;
            case EXTEND_PLACE_BEHIND:
                extendPlaceBehind();
                toOuttakeState = ToOuttakeState.NOTHING;
                break;
            case RETRACT:
                retractFromPlaceBehind();
                toOuttakeState = ToOuttakeState.NOTHING;
                break;
        }

        //outtake code
        if (targetV4BarPos != actualV4BarPos) {
            actualV4BarPos = targetV4BarPos;

            hardwareQueue.add(() -> leftOuttakeServo.setPosition(actualV4BarPos));
            hardwareQueue.add(() -> rightOuttakeServo.setPosition(actualV4BarPos));
        }

        if (Math.abs(targetWristPitch-actualWristPitch)>.05) {
            actualWristPitch = targetWristPitch;
            hardwareQueue.add(() -> wristPitchServo.setPosition(actualWristPitch));
        }

        if (Math.abs(targetWristRoll-actualWristRoll)>.05) {
            actualWristRoll = targetWristRoll;
            hardwareQueue.add(() -> wristRollServo.setPosition(actualWristRoll));
        }

        if (updateClawPosition) {
            hardwareQueue.add(() -> clawServo.setPosition(clawPosition.pos));
            updateClawPosition = false;
        }

        if(updatedHangPos) {
            hardwareQueue.add(() -> hangDeploy.setPosition(hangDeployPos.pos));
            updatedHangPos = false;
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

        if (targetSlidePos == 0) {
            f = .13;
        } else {
            f = 0;
        }

        slideI += error*elapsedTime;

        //Checks if error is in acceptable amounts

        if (absError>3) {
            //Slides set to max power
            p = Math.signum(error);
        } else {
            p = error*.2;
            d = ((error - prevSlideError) / elapsedTime) * 0;//.007
        }

        double motorPower =  p  - d + f; //
        slideTimer.reset();
        prevSlideError = error;

        hardwareQueue.add(() -> verticalRightMotor.setPower(motorPower));
        hardwareQueue.add(() -> verticalLeftMotor.setPower(motorPower));


        switch (outtakeState) {
            case EXTENDING_PLACE_BEHIND:
                if (absError<.5) {
                    outtakeState = OuttakeState.PLACING_BEHIND;
                }
                break;
            case PLACING_BEHIND:
                //claw opened by other code calling method
                if (clawPosition == ClawPosition.OPEN && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PLACING_BEHIND_RETRACT_DELAY;
                }
                break;
            case PLACING_BEHIND_RETRACT_DELAY:
                if (outtakeTimer.seconds()>.3) {
                    retractFromPlaceBehind();
                }
                break;
            case RETRACT_FROM_PLACE_BEHIND_DELAY:
                if (outtakeTimer.seconds()>.2) {
                    targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

                    outtakeState = OuttakeState.RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B;
                }
                break;
            case RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B:
                if (outtakeTimer.seconds()>1.4) {
                    targetSlidePos = VerticalSlide.WAIT_FOR_TRANSFER.length;

                    outtakeState = OuttakeState.RETRACT_FROM_PLACE_BEHIND;
                }
                break;
            case RETRACT_FROM_PLACE_BEHIND:
                if (absError<.5) {
                    outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                }
                break;
            case EXTENDING_PLACE_FRONT:
                if (absError<.5) {
                    outtakeState = OuttakeState.PLACING_FRONT;
                }
                break;
            case PLACING_FRONT:
                //claw opened by other code calling method
                if (place) {
//                    clawPosition = ClawPosition.OPEN;
                    targetSlidePos = VerticalSlide.PLACE_SPECIMEN_BAR.length;

                    place = false;
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.WAIT_PLACING_FRONT;
                }
                break;
            case WAIT_PLACING_FRONT:
                if (outtakeTimer.seconds()>.5) {
                    clawPosition = ClawPosition.OPEN;

                    outtakeState = OuttakeState.PLACING_FRONT_RETRACT_DELAY;
                    outtakeTimer.reset();
                }
                break;
            case PLACING_FRONT_RETRACT_DELAY:
                if (outtakeTimer.seconds()>.5) {
//                    clawPosition = ClawPosition.OPEN;
                    retractFromPlaceBehind();
//                    grabBehind();
                }
                break;
            case RETRACTING_GRAB_BEHIND_CLEAR_V4B:
                if (outtakeTimer.seconds()>1) {
                    retractFromPlaceBehind();
//                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length;
//
//                    outtakeState = OuttakeState.RETRACTING_GRAB_BEHIND;
                }
                break;
            case RETRACTING_GRAB_BEHIND:
                if (absError<.5) {
                    outtakeState = OuttakeState.WAITING_GRAB_BEHIND;
                }
                break;
            case WAITING_GRAB_BEHIND:
                if (clawPosition == ClawPosition.CLOSED && autoExtendSlides) {
                    updateClawPosition = true;
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.GRABBING_BEHIND_EXTEND_DELAY;
                }
                break;
            case GRABBING_BEHIND_EXTEND_DELAY:
                if (outtakeTimer.seconds()>.3) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length+2;
                    outtakeState = OuttakeState.LIFT_SPECIMEN;
                }
            case LIFT_SPECIMEN:
                if (absError<.5) {
                    //might need to do something to make sure it won't hit wall
                    extendPlaceFront();

//                    if (autoExtendSlides) {
//                        extendPlaceFront();
//                    } else {
//                        outtakeState = OuttakeState.IDLE;
//                    }
                }
                break;
            case WAITING_FOR_TRANSFER:
                if (grabFromTransfer) {
                    grabFromTransfer = false;

                    targetV4BarPos = V4BarPos.TRANSFER.pos;
                    targetWristPitch = WristPitch.DOWN.pos;
                    targetWristRoll = WristRoll.ZERO.pos;

                    targetSlidePos = VerticalSlide.TRANSFER.length;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.PICK_UP_FROM_TRANSFER;
                }
                break;
            case PICK_UP_FROM_TRANSFER:
                //waits for claw to be in position before grabbing sample
                if (outtakeTimer.seconds()>.5) {
                    clawPosition = ClawPosition.CLOSED;
                    updateClawPosition = true;

                    outtakeState = OuttakeState.WAIT_FOR_PICK_UP_FROM_TRANSFER;
                    outtakeTimer.reset();
                }
                break;
            case WAIT_FOR_PICK_UP_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.3) {
                    if (autoExtendSlides) {
                        extendPlaceBehind();
                    } else {
                        targetV4BarPos = V4BarPos.WAIT_FOR_TRANSFER.pos;
                        targetWristPitch = WristPitch.DOWN.pos;
                        targetWristRoll = WristRoll.ZERO.pos;

                        targetSlidePos = VerticalSlide.WAIT_FOR_TRANSFER.length;

                        outtakeState = OuttakeState.IDLE;
                    }
                }

                break;
            case IDLE:

                break;
            case INIT_POSITION:

                break;
        }

    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        V4BTelemetry.setValue(actualV4BarPos);

        outtakeSlideTelemetry.setValue(targetSlidePos);

        return super.dashboard(packet);
    }

    private double ticksToInches(int ticks) {
        return (ticks/384.5)*4.72;
    }

    private void extendPlaceBehind() {
        targetV4BarPos = V4BarPos.PLACE_BACK.pos;
        targetWristPitch = WristPitch.BACK.pos;
        targetWristRoll = WristRoll.NINETY.pos;

        targetSlidePos = VerticalSlide.HIGH_BUCKET.length;

        outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
    }

    private void extendPlaceFront() {
        targetV4BarPos = V4BarPos.PLACE_FRONT.pos;
        targetWristPitch = WristPitch.Front_ANGLED.pos;
        targetWristRoll = WristRoll.NINETY.pos;

        targetSlidePos = VerticalSlide.SPECIMEN_BAR.length;

        outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;
    }

//    private void extend

    private void grabBehind() {
        targetV4BarPos = V4BarPos.GRAB_BACK.pos;
        targetWristPitch = WristPitch.BACK.pos;
        targetWristRoll = WristRoll.NEGATIVE_NINETY.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        //set to this so V4B has room to rotate, set lower after v4b is clear
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACTING_GRAB_BEHIND_CLEAR_V4B;
    }

    private void retractFromPlaceBehind() {
        targetWristPitch = WristPitch.WAIT_FOR_TRANSFER.pos;
        targetWristRoll = WristRoll.ZERO.pos;
        targetV4BarPos = V4BarPos.WAIT_FOR_TRANSFER.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        //set to this so V4B has room to rotate, set lower after v4b is clear

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACT_FROM_PLACE_BEHIND_DELAY;
    }


    public void toOuttakeState(ToOuttakeState outtakeState) {
        targetToOuttakeState = outtakeState;
        updateToOuttakeState = true;
    }

    public void setTargetSlidePos(VerticalSlide targetPos) {
        setTargetSlidePos(targetPos.length);
    }

    public double getTargetSlidePos() {
        return prevTargetSlidePos;
    }

    public void setTargetSlidePos(double targetPos) {
        changedTargetSlidePos = true;
        newTargetSlidePos = MathUtil.clip(targetPos, -.5, 28.1);//28.35 in is max
    }

    public void setTargetV4BarPos(V4BarPos pos) {
        setTargetV4BarPos(pos.pos);
    }

    public void setTargetV4BarPos(double pos) {
        changedV4BarPos = true;
        newV4BarPos = pos;
    }

    public void setTargetWristPitch(WristPitch pos) {
        setTargetWristPitch(pos.pos);
    }

    public void setTargetWristPitch(double pos) {
        changedWristPitch = true;
        newWristPitch = pos;
    }

    public void setTargetWristRoll(WristRoll pos) {
        setTargetWristRoll(pos.pos);
    }

    public void setTargetWristRoll(double pos) {
        changedWristRoll = true;
        newWristRoll = pos;
    }

    public void setClawPosition(ClawPosition pos) {
        changedClawPosition = true;
        newClawPosition = pos;
    }

    public ClawPosition getTargetClawPosition() {
        return newClawPosition;
    }

    public void grabFromTransfer() {
        if (outtakeState == OuttakeState.WAITING_FOR_TRANSFER) {
            changedGrabFromTransfer = true;
        }
    }

    public void setHangDeploy(HangDeploy pos) {
        changedHangDeploy = true;
        newHangDeploy = pos;
    }

    public void place() {
        changedPlace = true;
    }


}
