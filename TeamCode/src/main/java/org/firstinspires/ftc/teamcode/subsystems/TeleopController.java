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
            intake.extendAndIntake(Intake.HorizontalSlide.MEDIUM);
        } else if (gamepad2.x) {
            intake.extendAndIntake(Intake.HorizontalSlide.FAR);
        } else if (gamepad2.right_bumper) {
            intake.retract();
        } else if (Math.abs(gamepad2.left_stick_y) > .05) {
            intake.setTargetSlidePos(intake.getTargetSlidePos() + 6 * loopTimer.seconds() * -gamepad2.left_stick_y);
        }


        if (gamepad2.dpad_right) {
            transfer.setTransferState(Transfer.TransferState.EJECT_RIGHT);
        } else if (gamepad2.dpad_left) {
            transfer.setTransferState(Transfer.TransferState.EJECT_LEFT);
        } else if (gamepad2.dpad_up) {
            transfer.setTransferState(Transfer.TransferState.CENTER);
        } else {
            transfer.setTransferState(Transfer.TransferState.NEUTRAL);

        }




//        if (gamepad2.b) {
//            outtake.
//        }

        loopTimer.reset();
    }
}
