package org.firstinspires.ftc.teamcode.subsystems;

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
        WAITING_CLEAR_BUCKET,
        RETRACTING_FROM_PLACE_BEHIND,

        EXTENDING_TO_DROP_SAMPLE,
        WAITING_DROP_SAMPLE,
        DROPPING_SAMPLE,
        MOVING_TO_GRAB_SPECIMEN,
        WAITING_GRAB_SPECIMEN,
        GRABBING_SPECIMEN,
        REMOVING_SPECIMEN_FROM_WALL,
        EXTENDING_PLACE_FRONT,
        WAITING_PLACE_FRONT,
        PLACING_FRONT,
        MOVING_TO_CLEAR_FRONT_BAR,
        MOVING_DOWN_TO_RETRACT,
        MOVING_ARM_BACK,
        RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE,

        START_RETRACTING_FROM_BEHIND,
        RETRACTING_FROM_BEHIND,

        WAITING_FOR_TRANSFER,
        GRABBING_FROM_TRANSFER,
        EXTRACTING_FROM_TRANSFER,
        VERIFYING_EXTRACTION,

        MOVING_TO_EJECTION,
        EJECTING,
        INIT_POSITION,
        IDLE
    }

    private OuttakeState outtakeState = OuttakeState.INIT_POSITION;

    public enum ToOuttakeState {
        WAIT_PLACE_FRONT,
        PLACE_FRONT,
        RETRACT_FROM_PLACE_BEHIND,
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
        TRANSFER(4),
        EXTRACT_FROM_TRANSFER(7),
        MIN_PASSTHROUGH_HEIGHT(8.5),
        SPECIMEN_PICKUP(1),
        CLEAR_SPECIMEN_BAR(4.7),
        SPECIMEN_BAR(8),
        PLACE_SPECIMEN_BAR(14.2),
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
        PLACE_FRONT(.53),
        CLEAR_FRONT_BAR(.72),
//        WAIT_FOR_TRANSFER(.35),
        MID_POSITION_CUTOFF(.2),
        TRANSFER(.285),
//        EXTRACT_FROM_TRANSFER(.35),
        EJECT_OUT_FRONT(.33),
        GRAB_BACK(.0),
        EXTRACT_FROM_GRAB_BACK(.1),
        PLACE_BACK(1),
        IDLE_POSITION(.5);

        public final double pos;

        V4BarPos(double pos) {
            this.pos = pos;
        }
    }

    private double targetV4BPos = V4BarPos.IDLE_POSITION.pos;

    private double actualV4BPos = targetV4BPos;//set to -1 so target will never == actual on first loop


    public enum ClawPitch {
        DOWN(.22),
        BACK(0.135),
        BACK_ANGLED_DOWN(.17),
        BACK2(.45),
        WAIT_FOR_TRANSFER(.3),
        TRANSFER(.26),
        EXTRACT_FROM_TRANSFER(.3),
        FRONT_ANGLED_UP(.3),
        FRONT_ANGELED_DOWN(.3),

        FRONT(.32);

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
        CLOSED(.1);

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

//    private boolean updateTransfer = false;

    private final boolean teleOpControls;
    private final boolean autoExtendSlides, autoRetractSlides;


    private final DcMotorEx verticalLeftMotor,verticalRightMotor;
    private final Encoder verticalSlideEncoder;

    private double actualMotorPower = 0;

    private boolean cycleSpecimen = false;

    private Gamepad oldGamePad2 = new Gamepad();

    private final NewIntake intake;

    private NewIntake.SampleColor sampleColor = NewIntake.SampleColor.NONE;

    private final boolean blueAlliance;

    private boolean intakeHoldingSample = false;
    private boolean updateIntakeHoldingSample = false;

    private double transferAttemptCounter = 0;

    private final double maxTransferAttempts = 2;

    private final Servo clawServo, clawPitchServo, leftOuttakeServo, rightOuttakeServo;

    public NewOuttake(SubSystemData data, NewIntake intake, boolean blueAlliance, boolean teleOpControls, boolean autoExtendSlides, boolean autoRetractSlides) {
        super(data);

        this.teleOpControls = teleOpControls;
        this.autoExtendSlides = autoExtendSlides;
        this.autoRetractSlides = autoRetractSlides;

        this.blueAlliance = blueAlliance;

        this.intake = intake;

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

        if (intake.transfer()) {
            transfer = true;
        }

        if (transfer) {
            sampleColor = intake.getSampleColor();
//            transfer = true;
//            updateTransfer = false;
        }

        if (updateIntakeHoldingSample) {
            intakeHoldingSample = intake.holdingSample();
            updateIntakeHoldingSample = false;
        }

    }

    @Override
    public void loop() {
        if (teleOpControls) {
            if (gamepad2.back) {
                if (gamepad2.y && !oldGamePad2.y) {
                    cycleSpecimen = false;
                } else if (gamepad2.b && !oldGamePad2.b) {
                    cycleSpecimen = true;
                }

                if (gamepad2.a && !oldGamePad2.a) {
                    verticalLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    targetSlidePos = 0;
                }

                if (Math.abs(gamepad2.right_stick_y) > .05) {
                    targetSlidePos = targetSlidePos+8 * slideTimer.seconds() * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75);
                }
            } else {
                if (gamepad2.right_trigger>.2 && oldGamePad2.right_trigger<=.2) {
                    transfer = true;
                } else if (gamepad2.a && !oldGamePad2.a) {
                    if (targetV4BPos>V4BarPos.MID_POSITION_CUTOFF.pos) {
                        toOuttakeState = ToOuttakeState.RETRACT_FROM_PLACE_BEHIND;
                    } else {
                        retractFromGrabBehind();
                    }
                } else if (gamepad2.b && !oldGamePad2.b) {
                    toOuttakeState = ToOuttakeState.PLACE_FRONT;
                } else if (gamepad2.x && !oldGamePad2.x) {
//                    if (clawPosition == ClawPosition.CLOSED) {
                        dropBehind();
//                    } else if (clawPosition == ClawPosition.OPEN) {
//                        clawPosition = ClawPosition.EXTRA_OPEN;
//                        updateClawPosition = true;

//                    }
                } else if (gamepad2.y && !oldGamePad2.y) {
                    toOuttakeState = ToOuttakeState.PLACE_BEHIND;
                } else if (Math.abs(gamepad2.right_stick_y) > .05) {
                    if (!gamepad2.back) {
                        targetSlidePos = MathUtil.clip(targetSlidePos+8 * slideTimer.seconds() * -gamepad2.right_stick_y * (1 - gamepad2.left_trigger * .75), -.5, 28.1);
                    } else {
//                    outtake.setTargetV4BarPos(outtake.getTargetV4BarPos()+gamepad2.right_stick_y * .0002 * loopTimer.milliSeconds());
                    }
                }
            }





            if (gamepad2.right_bumper && !oldGamePad2.right_bumper) {
                updateClawPosition = true;
                clawPosition = clawPosition == ClawPosition.CLOSED ? (outtakeState == OuttakeState.WAITING_DROP_SAMPLE ? ClawPosition.EXTRA_OPEN : ClawPosition.OPEN) : ClawPosition.CLOSED;
            }
        }


        switch (toOuttakeState) {
            case PLACE_BEHIND:
                extendPlaceBehind();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case PLACE_FRONT:
                extendPlaceFront();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
            case RETRACT_FROM_PLACE_BEHIND:
                retractFromFront();

                toOuttakeState = ToOuttakeState.IDLE;
                break;
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

        if (Math.abs(motorPower-actualMotorPower)>.04) {
            hardwareQueue.add(() -> verticalLeftMotor.setPower(motorPower));
            hardwareQueue.add(() -> verticalRightMotor.setPower(motorPower));

            actualMotorPower = motorPower;
        }


        if (Math.abs(targetClawPitch-actualClawPitch)>.01) {
            hardwareQueue.add(() -> clawPitchServo.setPosition(targetClawPitch));

            actualClawPitch = targetClawPitch;
        }

        if (Math.abs(targetV4BPos-actualV4BPos)>.04) {
            hardwareQueue.add(() -> leftOuttakeServo.setPosition(targetV4BPos));
            hardwareQueue.add(() -> rightOuttakeServo.setPosition(targetV4BPos));

            actualV4BPos = targetV4BPos;
        }

        if (updateClawPosition) {
            hardwareQueue.add(() -> clawServo.setPosition(clawPosition.pos));
        }



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
                if (outtakeTimer.seconds()>.1) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.WAITING_CLEAR_BUCKET;
                }
                break;
            case WAITING_CLEAR_BUCKET:
                if (outtakeTimer.seconds()>.3) {
                    retractFromFront();
                }
                break;
            case RETRACTING_FROM_PLACE_BEHIND:
                if (outtakeTimer.seconds()>.3 && absError<.5) {
                    outtakeState = OuttakeState.WAITING_FOR_TRANSFER;
                }
                break;


            case RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE:
                if (outtakeTimer.seconds()>.4) {
                    targetClawPitch = ClawPitch.BACK_ANGLED_DOWN.pos;

                    outtakeTimer.reset();
                    outtakeState = OuttakeState.EXTENDING_TO_DROP_SAMPLE;
                }
                break;
            case EXTENDING_TO_DROP_SAMPLE:
                if (outtakeTimer.seconds()>.5) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length;

                    outtakeState = OuttakeState.WAITING_DROP_SAMPLE;
                }
                break;
            case WAITING_DROP_SAMPLE:
                if (clawPosition == ClawPosition.OPEN || clawPosition == ClawPosition.EXTRA_OPEN) {
                    if (clawPosition != ClawPosition.EXTRA_OPEN) {
                        clawPosition = ClawPosition.EXTRA_OPEN;
                        updateClawPosition = true;
                    }

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.DROPPING_SAMPLE;
                }
                 break;
            case DROPPING_SAMPLE:
                if (outtakeTimer.seconds()>.2) {
                    targetClawPitch = ClawPitch.BACK.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_TO_GRAB_SPECIMEN;
                }
                break;
            case MOVING_TO_GRAB_SPECIMEN:
                if (outtakeTimer.seconds()>.2) {
                    outtakeState = OuttakeState.WAITING_GRAB_SPECIMEN;
                }
                break;
            case WAITING_GRAB_SPECIMEN:
                if (clawPosition == ClawPosition.CLOSED && autoRetractSlides) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.GRABBING_SPECIMEN;
                }
                break;
            case GRABBING_SPECIMEN:
                if (outtakeTimer.seconds()>.3) {
                    targetSlidePos = VerticalSlide.SPECIMEN_PICKUP.length+4;
//                    targetV4BPos = V4BarPos.EXTRACT_FROM_GRAB_BACK.pos;
//                    targetClawPitch = ClawPitch.EXTRACT_FROM_TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.REMOVING_SPECIMEN_FROM_WALL;
                }
                break;

            case REMOVING_SPECIMEN_FROM_WALL:
                if (outtakeTimer.seconds()>.2) {
                    extendPlaceFront();
                }
                break;
//            case EXTENDING_CLEAR_TRANSFER:
//                if (slidePos>VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length) {
//                    targetV4BPos = V4BarPos.PLACE_BACK.pos;
//
//                    outtakeTimer.reset();
//
//                    outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
//                }
//                break;
            case EXTENDING_PLACE_FRONT:
                if (absError<.5) {
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
                if (outtakeTimer.seconds()>.1) {
                    targetV4BPos = V4BarPos.CLEAR_FRONT_BAR.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_TO_CLEAR_FRONT_BAR;
                }
                break;
            case MOVING_TO_CLEAR_FRONT_BAR:
                if (outtakeTimer.seconds()>.3) {
                    targetSlidePos = VerticalSlide.CLEAR_SPECIMEN_BAR.length;

                    outtakeState = OuttakeState.MOVING_DOWN_TO_RETRACT;
                }
                break;
            case MOVING_DOWN_TO_RETRACT:
                if (absError<.5) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;
                    outtakeTimer.reset();

                    outtakeState = OuttakeState.MOVING_ARM_BACK;
                }
                break;
            case MOVING_ARM_BACK:
                if (outtakeTimer.seconds()>.4) {
                    retractFromFront();
                }
                break;

            case START_RETRACTING_FROM_BEHIND:
                if (outtakeTimer.seconds()>.3) {
                    targetV4BPos = V4BarPos.TRANSFER.pos;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.RETRACTING_FROM_BEHIND;
                }
                break;
            case RETRACTING_FROM_BEHIND:
                if (outtakeTimer.seconds()>.3) {
                    clawPosition = ClawPosition.OPEN;
                    updateClawPosition = true;

                    targetSlidePos = VerticalSlide.TRANSFER.length;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.RETRACTING_FROM_PLACE_BEHIND;
                }
                break;


            case WAITING_FOR_TRANSFER:
                if (transfer) {
                    transfer = false;

                    clawPosition = ClawPosition.CLOSED;
                    updateClawPosition = true;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.GRABBING_FROM_TRANSFER;
                }
                break;
            case GRABBING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.2) {
                    targetSlidePos = VerticalSlide.EXTRACT_FROM_TRANSFER.length;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EXTRACTING_FROM_TRANSFER;
                }
                break;
            case EXTRACTING_FROM_TRANSFER:
                if (outtakeTimer.seconds()>.3) {
                    updateIntakeHoldingSample = true;
                    outtakeState = OuttakeState.VERIFYING_EXTRACTION;
                }
                break;
            case VERIFYING_EXTRACTION:
                    //trys to grab sample again if first grab fails
                    if (intakeHoldingSample) {//intakeHoldingSample
                        if (transferAttemptCounter == 0) {
                            retractFromFront();

                            intake.setIntakingState(NewIntake.IntakingState.START_REINTAKING);
                            intake.setIntakeState(NewIntake.IntakeState.WAITING_FOR_TRANSFER);

                            transferAttemptCounter++;
                        } else if (transferAttemptCounter < maxTransferAttempts) {
                            retractFromFront();
                            intake.setIntakingState(NewIntake.IntakingState.START_UNJAMMING);
                            intake.setIntakeState(NewIntake.IntakeState.WAITING_FOR_TRANSFER);

                            transferAttemptCounter++;
                        } else {
                            retractFromFront();

                            transferAttemptCounter = 0;
                        }

                        break;
                    }

                    transferAttemptCounter = 0;

                    if ((sampleColor == NewIntake.SampleColor.RED && blueAlliance) || (sampleColor == NewIntake.SampleColor.BLUE && !blueAlliance)) {
                        targetSlidePos = VerticalSlide.TRANSFER.length + 2;
                        targetV4BPos = V4BarPos.EJECT_OUT_FRONT.pos;
                        targetClawPitch = ClawPitch.FRONT_ANGELED_DOWN.pos;

                        outtakeTimer.reset();

                        outtakeState = OuttakeState.MOVING_TO_EJECTION;
                    } else if (autoExtendSlides) {
                        if (cycleSpecimen && sampleColor != NewIntake.SampleColor.YELLOW) {
                            dropBehind();
                        } else {
                            extendPlaceBehind();
                        }
                    } else {
                        outtakeState = OuttakeState.IDLE;
                    }
                break;


            case MOVING_TO_EJECTION:
                if (outtakeTimer.seconds()>.3) {
                    clawPosition = ClawPosition.OPEN;
                    updateClawPosition = true;

                    outtakeTimer.reset();

                    outtakeState = OuttakeState.EJECTING;
                }
                break;
            case EJECTING:
                if (outtakeTimer.seconds()>.2) {
                    retractFromFront();
                }
                break;
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
        targetSlidePos = VerticalSlide.HIGH_BUCKET.length;

        targetV4BPos = V4BarPos.PLACE_BACK.pos;

        targetClawPitch = ClawPitch.BACK2.pos;

        outtakeState = OuttakeState.EXTENDING_PLACE_BEHIND;
    }

    private void extendPlaceFront() {
        targetSlidePos = VerticalSlide.PLACE_SPECIMEN_BAR.length;

        targetV4BPos = V4BarPos.PLACE_FRONT.pos;

        targetClawPitch = ClawPitch.DOWN.pos;

        outtakeState = OuttakeState.EXTENDING_PLACE_FRONT;
    }

//    private void extend

    private void dropBehind() {
        targetV4BPos = V4BarPos.GRAB_BACK.pos;
        targetClawPitch = ClawPitch.FRONT.pos;

        //set to this so V4B has room to rotate, set lower after v4b is clear
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        outtakeTimer.reset();

        outtakeState = OuttakeState.RETRACING_FROM_PLACE_FRONT_CLEAR_INTAKE;
    }

    private void retractFromFront() {
        targetSlidePos = VerticalSlide.TRANSFER.length;

        targetV4BPos = V4BarPos.TRANSFER.pos;

        targetClawPitch = ClawPitch.TRANSFER.pos;

        if (clawPosition != ClawPosition.OPEN) {
            clawPosition = ClawPosition.OPEN;
            updateClawPosition = true;
        }

        outtakeState = OuttakeState.RETRACTING_FROM_PLACE_BEHIND;

        outtakeTimer.reset();
    }

    private void retractFromGrabBehind() {
        targetSlidePos = VerticalSlide.MIN_PASSTHROUGH_HEIGHT.length;

        targetClawPitch = ClawPitch.TRANSFER.pos;

        if (clawPosition != ClawPosition.CLOSED) {
            clawPosition = ClawPosition.CLOSED;
            updateClawPosition = true;
        }

        outtakeTimer.reset();

        outtakeState = OuttakeState.START_RETRACTING_FROM_BEHIND;
    }

//    public void transfer() {
//        updateTransfer = true;
//    }
}