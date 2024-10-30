package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class TeleopController extends SubSystem {

    private enum CycleMode {
        NORMAL,
        SPECIMEN
    }

    private boolean grabFromTransfer = false;

    CycleMode cycleMode = CycleMode.NORMAL;

    Intake intake;
    Transfer transfer;
    Outtake outtake;

    ElapsedTimer loopTimer = new ElapsedTimer();

    Gamepad oldGamePad1 = new Gamepad();
    Gamepad oldGamePad2 = new Gamepad();


    public TeleopController(Intake intake, Transfer transfer, Outtake outtake, SubSystemData data) {
        super(data);

        this.intake = intake;
        this.transfer = transfer;
        this.outtake = outtake;

        intake.setTargetIntakePos(Intake.IntakePos.PARTIAL_UP);
        transfer.setTransferState(Transfer.TransferState.NEUTRAL);


    }

    @Override
    public void priorityData() {

    }

    @Override
    public void loop() {

        //Intake drop and intake controls
        if (gamepad2.left_bumper && !oldGamePad2.left_bumper) {
            intake.setTargetIntakePos(Intake.IntakePos.DOWN);
            intake.setTargetIntakeSpeed(1);
        } else if (gamepad2.right_trigger>.1) {
            intake.setTargetIntakeSpeed(-1);
        } else if ((!gamepad2.left_bumper && oldGamePad2.left_bumper) || (gamepad2.right_trigger<=.1 && oldGamePad2.right_trigger>.1)) {
            intake.setTargetIntakeSpeed(0);
        }


        if (gamepad2.back && !oldGamePad2.back) {
            intake.setTargetIntakePos(Intake.IntakePos.PARTIAL_UP);
        }


        //Horizontal slides
        if (gamepad2.dpad_down && !oldGamePad2.dpad_down) {
            intake.retract();
            transfer.setTransferState(Transfer.TransferState.NEUTRAL);
        } else if (gamepad2.dpad_right && !oldGamePad2.dpad_right) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.CLOSE);
        } else if (gamepad2.dpad_left && !oldGamePad2.dpad_left) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.MEDIUM);
        } else if (gamepad2.dpad_up && !oldGamePad2.dpad_up) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.FAR);
        } else if (Math.abs(gamepad2.left_stick_y) > .05) {
            intake.setTargetSlidePos(intake.getTargetSlidePos() + 10 * loopTimer.seconds() * -gamepad2.left_stick_y * (1-gamepad2.right_trigger*.75));
        }



        //Claw code
        if (gamepad2.right_bumper && !oldGamePad2.right_bumper) {
            outtake.setClawPosition(outtake.getTargetClawPosition() == Outtake.ClawPosition.OPEN ? Outtake.ClawPosition.CLOSED : Outtake.ClawPosition.OPEN);
        }

        //Vertical Slides
        if (gamepad2.a && !oldGamePad2.a) {
            outtake.toOuttakeState(Outtake.ToOuttakeState.RETRACT);
        } else if (gamepad2.b && !oldGamePad2.b) {
            outtake.toOuttakeState(Outtake.ToOuttakeState.GRAB_BEHIND);
        } else if (gamepad2.x && !oldGamePad2.x) {
            outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_FRONT);
        } else if (gamepad2.y && !oldGamePad2.y) {
            outtake.toOuttakeState(Outtake.ToOuttakeState.EXTEND_PLACE_BEHIND);
        } else if (Math.abs(gamepad2.right_stick_y) > .05) {
            outtake.setTargetSlidePos(outtake.getTargetSlidePos() + 8 * loopTimer.seconds() * -gamepad2.right_stick_y * (1-gamepad2.left_trigger*.75));
        }


        //Transfer
        if (intake.transfered()) {
            transfer.setTransferState(Transfer.TransferState.CENTER);

            if (cycleMode == CycleMode.NORMAL) {
                grabFromTransfer = true;
            }
        }

        if (grabFromTransfer && gamepad2.a) {
            outtake.grabFromTransfer();
            grabFromTransfer = false;
        }

        if (gamepad1.left_bumper && !oldGamePad1.left_bumper) {
            transfer.setTransferState(Transfer.TransferState.EJECT_LEFT);
            cycleMode = CycleMode.SPECIMEN;
        } else if (gamepad1.right_bumper && !oldGamePad1.right_bumper) {
            transfer.setTransferState(Transfer.TransferState.EJECT_RIGHT);
            cycleMode = CycleMode.SPECIMEN;
        } else if ((!gamepad1.left_bumper && oldGamePad1.left_bumper) || (!gamepad1.right_bumper && oldGamePad1.right_bumper)) {
            transfer.setTransferState(Transfer.TransferState.NEUTRAL);
        }

        if (gamepad1.dpad_up && !oldGamePad1.dpad_up) {
            cycleMode = CycleMode.NORMAL;
        }



        oldGamePad1.copy(gamepad1);
        oldGamePad2.copy(gamepad2);

        loopTimer.reset();
    }
}
