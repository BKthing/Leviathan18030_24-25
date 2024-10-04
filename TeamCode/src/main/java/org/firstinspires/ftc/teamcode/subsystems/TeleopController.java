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
        } else if (gamepad1.x) {
            intake.extendAndIntake(Intake.HorizontalSlide.FAR);
        } else if (gamepad2.right_bumper) {
            intake.retract();
        } else if (Math.abs(gamepad1.left_stick_y) > .05) {
            intake.setTargetSlidePos(intake.getTargetSlidePos()+.01*loopTimer.seconds());
        }

//        if (gamepad2.b) {
//            outtake.
//        }

        loopTimer.reset();
    }
}
