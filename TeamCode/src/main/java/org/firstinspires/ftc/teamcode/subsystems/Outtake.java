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

public class Outtake extends SubSystem {

    public enum OuttakeState {
        EXTENDING_PLACE_BEHIND,
        PLACING_BEHIND,
        RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B,
        RETRACT_FROM_PLACE_BEHIND,
        EXTENDING_PLACE_FRONT,
        PLACING_FRONT,
        RETRACTING_GRAB_BEHIND_CLEAR_V4B,
        RETRACTING_GRAB_BEHIND,
        WAITING_GRAB_BEHIND,
        LIFT_SPECIMEN,
        WAITING_FOR_TRANSFER,
        PICK_UP_FROM_TRANSFER,
        IDLE,

        INIT_POSITION
    }

    OuttakeState outtakeState = OuttakeState.INIT_POSITION;
    OuttakeState newOuttakeState = OuttakeState.INIT_POSITION;


    boolean changedOuttakeState = false;

    ElapsedTimer outtakeTimer = new ElapsedTimer();


    public enum VerticalSlide {
        //28.35in max
        BOTTOM(0),
        WAIT_FOR_TRANSFER(5),
        TRANSFER(4),
        MIN_PASSTHROUGH_HEIGHT(8),
        SPECIMEN_PICKUP(6),
        SPECIMEN_BAR(16),
        LOW_BUCKET_HEIGHT(20),
        HIGH_BUCKET(28);

        public final double length;
        VerticalSlide(double length) {this.length = length;}
    }



    ElapsedTimer slideTimer = new ElapsedTimer();

    boolean changedTargetSlidePos = false;

    double targetSlidePos;
    double newTargetSlidePos;
    double prevTargetSlidePos;

    double slidePos;//inches
    double prevSlideError = 0;

    double slideI = 0;

    int slideTicks = 0;

    Encoder verticalSlideEncoder;

    DcMotorEx verticalLeftMotor, verticalRightMotor;


    public enum V4BarPos {
        PLACE_FRONT(0),
        WAIT_FOR_TRANSFER(.3),
        TRANSFER(.33),
        GRAB_BACK(.55),
        PLACE_BACK(.7);

        public final double pos;

        V4BarPos(double pos) {
            this.pos = pos;
        }
    }
    boolean changedV4BarPos = false;
    double targetV4BarPos = 0;
    double actualV4BarPos = 0;
    double newV4BarPos = 0;

    Servo leftOuttakeServo, rightOuttakeServo;


    public enum WristPitch {
        DOWN(.4),
        BACK(0.6),
        BACK_ANGLED(.7),
        FRONT(.1);

        public final double pos;

        WristPitch(double pos) {
            this.pos = pos;
        }
    }
    boolean changedWristPitch = false;
    double targetWristPitch = 0;
    double actualWristPitch = 0;
    double newWristPitch = 0;

    Servo wristPitchServo;


    public enum WristRoll {
        NEGATIVE_NINETY(0),
        ZERO(.33),
        NINETY(0.66),
        TWOSEVENTY(1);

        public final double pos;

        WristRoll(double pos) {
            this.pos = pos;
        }
    }

    boolean changedWristRoll = false;
    double targetWristRoll = 0;
    double actualWristRoll = 0;
    double newWristRoll = 0;

    Servo wristRollServo;


    public enum ClawPosition {
        OPEN(.3),
        CLOSED(.6);

        public final double pos;

        ClawPosition(double pos) {
            this.pos = pos;
        }
    }

    ClawPosition clawPosition = ClawPosition.CLOSED;
    ClawPosition newClawPosition = ClawPosition.CLOSED;

    boolean updatedClawPosition = false;

    boolean changedClawPosition = false;

    boolean changedHangDeploy = false;

    boolean updatedHangPos = false;

    Servo clawServo;

    boolean grabFromTransfer = false;
    boolean changedGrabFromTransfer = false;


    boolean autoExtendSlides = true;
    boolean toggleAutoExtendSlides = false;

    Servo hangDeploy;

    public enum HangDeploy {
        DEPLOY(.4),
        NOTDEPLOYED(.2);

        public final double pos;

        HangDeploy(double pos) {this.pos = pos;}
    }

    HangDeploy hangDeployPos = HangDeploy.NOTDEPLOYED;
    HangDeploy newHangDeploy = HangDeploy.NOTDEPLOYED;


    public Outtake(SubSystemData data) {
        super(data);

        //Motors
        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        verticalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft"); // control hub 2
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight"); // control hub 3

//        verticalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servos
        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo");

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);

        wristPitchServo = hardwareMap.get(Servo.class, "wristPitchServo");
        wristRollServo = hardwareMap.get(Servo.class, "wristRollServo");

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        hangDeploy = hardwareMap.get(Servo.class, "hangDeploy");


        //initiating slide encoder
        slidePos = ticksToInches(verticalSlideEncoder.getCurrentPosition());

        if (PassData.horizontalSlidesInitiated && slidePos>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePos>1) {
                targetSlidePos = slidePos;
            }
        } else {
            verticalRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slidePos = 0;
            targetSlidePos = 0;

            PassData.horizontalSlidesInitiated = true;
        }

        newTargetSlidePos = targetSlidePos;


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
            updatedClawPosition = true;
            changedClawPosition = false;
        }

        if (changedGrabFromTransfer) {
            grabFromTransfer = true;
        }

        if (changedHangDeploy) {
            hangDeployPos = newHangDeploy;
            updatedHangPos = true;
            changedHangDeploy = false;
        }


        if (toggleAutoExtendSlides) {
            autoExtendSlides = !autoExtendSlides;
            toggleAutoExtendSlides = false;
        }

        prevTargetSlidePos = targetSlidePos;
    }

    @Override
    public void loop() {
        //outtake code
        if (Math.abs(targetV4BarPos-actualV4BarPos)>.05) {
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

        if (updatedClawPosition) {
            hardwareQueue.add(() -> clawServo.setPosition(clawPosition.pos));
            updatedClawPosition = false;
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

        if (absError<0) {
            //Slides set to max power
            p = Math.signum(error);
        } else if (absError<0) {
            //Slides set to max power
            p = Math.signum(error)*.5;
        } else {//if (error<4 but error>.1)
            p = error*.12;
            d = ((error - prevSlideError) / elapsedTime) * 0;//.007
        }

        double motorPower =  p + slideI - d + f; //
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
                if (clawPosition == ClawPosition.OPEN) {

                    targetV4BarPos = V4BarPos.WAIT_FOR_TRANSFER.pos;
                    targetWristPitch = WristPitch.DOWN.pos;
                    targetWristRoll = WristRoll.ZERO.pos;

                    //set to this so V4B has room to rotate, set lower after v4b is clear
                    targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

                    slideTimer.reset();

                    outtakeState = OuttakeState.RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B;
                }
                break;
            case RETRACT_FROM_PLACE_BEHIND_CLEAR_V4B:
                if (slideTimer.seconds()>1) {
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
                if (clawPosition == ClawPosition.OPEN) {

                    targetV4BarPos = V4BarPos.GRAB_BACK.pos;
                    targetWristPitch = WristPitch.BACK.pos;
                    targetWristRoll = WristRoll.NEGATIVE_NINETY.pos;

                    //set to this so V4B has room to rotate, set lower after v4b is clear
                    targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

                    slideTimer.reset();

                    outtakeState = OuttakeState.RETRACTING_GRAB_BEHIND_CLEAR_V4B;
                }
                break;
            case RETRACTING_GRAB_BEHIND_CLEAR_V4B:
                if (slideTimer.seconds()>1) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length;

                    outtakeState = OuttakeState.RETRACTING_GRAB_BEHIND;
                }
                break;
            case RETRACTING_GRAB_BEHIND:
                if (absError<.5) {
                    outtakeState = OuttakeState.WAITING_GRAB_BEHIND;
                }
                break;
            case WAITING_GRAB_BEHIND:
                if (clawPosition == ClawPosition.CLOSED) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length+2;
                    outtakeState = OuttakeState.LIFT_SPECIMEN;
                }
                break;
            case LIFT_SPECIMEN:
                if (absError<.5) {
                    //might need to do something to make sure it won't hit wall
                    if (autoExtendSlides) {
                        extendPlaceFront();
                    } else {
                        outtakeState = OuttakeState.IDLE;
                    }
                }
                break;
            case WAITING_FOR_TRANSFER:
                if (grabFromTransfer) {
                    grabFromTransfer = false;

                    targetV4BarPos = V4BarPos.TRANSFER.pos;
                    targetWristPitch = WristPitch.DOWN.pos;
                    targetWristRoll = WristRoll.ZERO.pos;

                    targetSlidePos = VerticalSlide.TRANSFER.length;

                    slideTimer.reset();

                    outtakeState = OuttakeState.PICK_UP_FROM_TRANSFER;
                }
                break;
            case PICK_UP_FROM_TRANSFER:
                //waits for claw to be in position before grabbing sample
                if (slideTimer.seconds()>.4) {
                    clawPosition = ClawPosition.CLOSED;

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
        targetWristPitch = WristPitch.FRONT.pos;
        targetWristRoll = WristRoll.NINETY.pos;

        targetSlidePos = VerticalSlide.SPECIMEN_BAR.length;

        outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;
    }


    public void setTargetSlidePos(VerticalSlide targetPos) {
        setTargetSlidePos(targetPos.length);
    }

    public double getTargetSlidePos() {
        return prevTargetSlidePos;
    }

    public void setTargetSlidePos(double targetPos) {
        changedTargetSlidePos = true;
        newTargetSlidePos = MathUtil.clip(targetPos, 0, 28.1);//28.35 in is max
    }

    public void setTargetV4BarPos(V4BarPos pos) {
        setTargetV4BarPos(pos.pos);
    }

    public void setTargetV4BarPos(double pos) {
        changedV4BarPos = true;
        newV4BarPos = pos;
    }

    public void setTargetWristPitch(WristPitch pos) {
        setTargetV4BarPos(pos.pos);
    }

    public void setTargetWristPitch(double pos) {
        changedWristPitch = true;
        newWristPitch = pos;
    }

    public void setTargetWristRoll(WristRoll pos) {
        setTargetV4BarPos(pos.pos);
    }

    public void setTargetWristRoll(double pos) {
        changedWristRoll = true;
        newWristRoll = pos;
    }

    public void setClawPosition(ClawPosition pos) {
        changedClawPosition = true;
        clawPosition = pos;
    }

    public void grabFromTransfer() {
        changedGrabFromTransfer = true;
    }

    public void setHangDeploy(HangDeploy pos) {
        changedHangDeploy = true;
        hangDeployPos = pos;
    }


}
