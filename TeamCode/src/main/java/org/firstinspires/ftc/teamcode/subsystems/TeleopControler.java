package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class TeleopControler extends SubSystem {

    Intake intake;
    Transfer transfer;
    Outtake outtake;


    public TeleopControler(Intake intake, Transfer transfer, Outtake outtake, SubSystemData data) {
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
        }

//        if (gamepad2.b) {
//            outtake.
//        }


    }
}
