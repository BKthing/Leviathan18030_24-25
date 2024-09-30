package org.firstinspires.ftc.teamcode.subsystems;

public class Intake extends SubSystem {

    public enum IntakeState {
        EXTENDING,
        INTAKING,
        RETRACTING,
        TRANSFERING,
        RESTING
    }

    IntakeState intakeState = IntakeState.RESTING;

    double slidePos = 0;//inches


}
