package org.firstinspires.ftc.teamcode.subsystems;

import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class TeleopController extends SubSystem {

    Intake intake;
    Transfer transfer;
    Outtake outtake;

    ElapsedTimer loopTimer = new ElapsedTimer();


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

        if (gamepad2.a) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.IN);
        } else if (gamepad2.b) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.CLOSE);
        }
        else if (gamepad2.x) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.MEDIUM);
        }
        else if (gamepad2.y) {
            intake.setTargetSlidePos(Intake.HorizontalSlide.FAR);

        } else if (gamepad2.right_bumper) {
            intake.retract();
        } else if (Math.abs(gamepad2.left_stick_y) > .05) {
            intake.setTargetSlidePos(intake.getTargetSlidePos() + 6 * loopTimer.seconds() * -gamepad2.left_stick_y);
        }

        if (Math.abs(gamepad2.right_stick_y) > .05) {
            outtake.setTargetSlidePos(outtake.getTargetSlidePos() + 6 * loopTimer.seconds() * -gamepad2.right_stick_y);
        }

        if (gamepad2.dpad_down) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.BOTTOM);
        }
        else if (gamepad2.dpad_right) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.SPECIMEN_BAR);
        }
        else if (gamepad2.dpad_left) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.LOW_BUCKET_HEIGHT);
        }
        else if (gamepad2.dpad_up) {
            outtake.setTargetSlidePos(Outtake.VerticalSlide.HIGH_BUCKET);
        }


//        outtake.setHangDeploy(Outtake.HangDeploy.DEPLOY);
//
//        if (gamepad2.dpad_right) {
//            transfer.setTransferState(Transfer.TransferState.EJECT_RIGHT);
//        } else if (gamepad2.dpad_left) {
//            transfer.setTransferState(Transfer.TransferState.EJECT_LEFT);
//        } else if (gamepad2.dpad_up) {
//            transfer.setTransferState(Transfer.TransferState.CENTER);
//        } else {
//            transfer.setTransferState(Transfer.TransferState.NEUTRAL);

//        }

//         if (gamepad2.left_bumper) {
//            intake.setTargetIntakeSpeed(1);
//        }
//         else if (gamepad2.right_bumper) {
//             intake.setTargetIntakeSpeed(-1);
//         }
//         else
//             intake.setTargetIntakeSpeed(0);
//
//         if (gamepad2.x) {
//             intake.setIntakePos(Intake.IntakePos.DOWN);
//         }
//         if (gamepad2.a) {
//             intake.setIntakePos(Intake.IntakePos.UP);
//         }
//         if (gamepad2.y) {
//             intake.setIntakePos(Intake.IntakePos.PARTIAL_UP);
//         }



//        else {
//            intake.leftIntakeServo.setPosition(0);
//            intake.rightIntakeServo.setPosition(0);
//            intake.intakeServo.setPosition(0);
//        }



        loopTimer.reset();
    }
}
