package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class TeleopController extends SubSystem {

    private enum CycleMode {
        NORMAL,
        SPECIMEN
    }

    CycleMode cycleMode = CycleMode.NORMAL;

    Intake intake;
    Transfer transfer;
    Outtake outtake;

    ElapsedTimer loopTimer = new ElapsedTimer();

    Gamepad oldGamePad2 = new Gamepad();


    public TeleopController(Intake intake, Transfer transfer, Outtake outtake, SubSystemData data) {
        super(data);

        this.intake = intake;
        this.transfer = transfer;
        this.outtake = outtake;

    }

    @Override
    public void priorityData() {

    }

    @Override
    public void loop() {

        //Intake drop and intake controls
        if (gamepad2.left_bumper && !oldGamePad2.left_bumper) {
            intake.setIntakePos(Intake.IntakePos.DOWN);
            intake.setTargetIntakeSpeed(1);
        } else if (!gamepad2.left_bumper && oldGamePad2.left_bumper) {
            intake.setTargetIntakeSpeed(0);
        }


        if (gamepad2.back && !oldGamePad2.back) {
            intake.setIntakePos(Intake.IntakePos.PARTIAL_UP);
        }


        //Horizontal slides
        if (gamepad2.dpad_down && !oldGamePad2.dpad_down) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.IN);
        } else if (gamepad2.dpad_right && !oldGamePad2.dpad_right) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.CLOSE);
        } else if (gamepad2.dpad_left && !oldGamePad2.dpad_left) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.MEDIUM);
        } else if (gamepad2.dpad_up && !oldGamePad2.dpad_up) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.FAR);
        } else if (Math.abs(gamepad2.left_stick_y) > .05) {
            intake.setTargetSlidePos(intake.getTargetSlidePos() + 8 * loopTimer.seconds() * -gamepad2.left_stick_y * (1-gamepad2.right_trigger*.75));
        }



        //Claw code
        if (gamepad2.right_bumper && !oldGamePad2.right_bumper) {
            outtake.setClawPosition(outtake.getTargetClawPosition() == Outtake.ClawPosition.OPEN ? Outtake.ClawPosition.CLOSED : Outtake.ClawPosition.OPEN);
        }

        //Vertical Slides
        if (gamepad2.a && !oldGamePad2.a) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.BOTTOM);
        } else if (gamepad2.b && !oldGamePad2.b) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_BAR);
        } else if (gamepad2.x && !oldGamePad2.x) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.LOW_BUCKET_HEIGHT);
        } else if (gamepad2.y && !oldGamePad2.y) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.HIGH_BUCKET);
        } else if (Math.abs(gamepad2.right_stick_y) > .05) {
            outtake.setTargetSlidePos(outtake.getTargetSlidePos() + 8 * loopTimer.seconds() * -gamepad2.right_stick_y * (1-gamepad2.left_trigger*.75));
        }


        //Vertical Slides
//        switch (cycleMode) {
//            case NORMAL:
//                if (gamepad2.a && !oldGamePad2.a) {
//                    outtake.setTargetSlidePos(Outtake.VerticalSlide.BOTTOM);
//                } else if (gamepad2.b && !oldGamePad2.b) {
//                    cycleMode = CycleMode.SPECIMEN;
//                } else if (gamepad2.y && !oldGamePad2.y) {
//                    outtake.setTargetSlidePos(Outtake.VerticalSlide.HIGH_BUCKET);
//                }
//                break;
//            case SPECIMEN:
//                if (gamepad2.a && !oldGamePad2.a) {
//                    outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_PICKUP);
//                } else if (gamepad2.x && !oldGamePad2.x) {
//                    cycleMode = CycleMode.NORMAL;
//                } else if (gamepad2.y && !oldGamePad2.y) {
//                    outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_BAR);
//                }
//                break;
//        }
//
//        if (Math.abs(gamepad2.right_stick_y) > .05) {
//            outtake.setTargetSlidePos(outtake.getTargetSlidePos() + 8 * loopTimer.seconds() * -gamepad2.right_stick_y * (1-gamepad2.left_trigger*.75));
//        }



        oldGamePad2.copy(gamepad2);

        loopTimer.reset();
    }
}
