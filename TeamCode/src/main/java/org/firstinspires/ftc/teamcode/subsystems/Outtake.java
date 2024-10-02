package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class Outtake extends SubSystem {

    public enum OuttakeState {
        EXTENDING_PLACE_BEHIND,
        PLACING_BEHIND,
        RETRACT_FROM_PLACE_BEHIND,

        EXTENDING_PLACE_FRONT,
        PLACING_FRONT,
        RETRACTING_GRAB_BEHIND,
        WAITING_GRAB_BEHIND,

        MOVING_FROM_BEHIND_TO_TRANSFER,

        WAITING_FOR_TRANSFER,
        PICK_UP_FROM_TRANSFER,

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
        SPECIMEN_BAR(16),
        LOW_BUCKET_HEIGHT(20),
        HIGH_BUCKET(28);

        public final double length;
        VerticalSlide(double length) {this.length = length;}
    }

    boolean changedTargetSlidePos = false;

    double targetSlidePos;
    double newTargetSlidePos;

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

    boolean changedClawPosition = false;

    Servo clawServo;


    public Outtake(SubSystemData data) {
        super(data);

        //Motors
        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));

        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft");
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight");

        verticalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Servos
        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo");

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);

        wristPitchServo = hardwareMap.get(Servo.class, "wristPitchServo");
        wristRollServo = hardwareMap.get(Servo.class, "wristRollServo");

        clawServo = hardwareMap.get(Servo.class, "clawServo");


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
            changedClawPosition = false;
        }
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
    }

    private double ticksToInches(int ticks) {
        return (ticks/145.1)*4.72;
    }


}
