package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class Transfer extends SubSystem {
    public enum TransferState {
        EJECT_LEFT(.46 , 0.23, .755),
        EJECT_RIGHT(0.7, .47, .945),
        NEUTRAL(0.7, 0.23, .84),
        CENTER(.795, 0.135, .84);

        public final double leftServoPos, rightServoPos, tiltServoPos;

        TransferState(double leftServoPos, double rightServoPos, double tiltServoPos) {
            this.leftServoPos = leftServoPos;
            this.rightServoPos = rightServoPos;
            this.tiltServoPos = tiltServoPos;
        }
    }

    TransferState transferState = TransferState.NEUTRAL;
    TransferState newTransferState = TransferState.NEUTRAL;

    boolean update = true;

    Servo leftServo, rightServo, tiltServo;

    public Transfer(SubSystemData data) {
        super(data);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
    }

    @Override
    public void priorityData() {
        if (transferState != newTransferState) {
            transferState = newTransferState;
            update = true;
        }
    }

    @Override
    public void loop() {
        if (update) {
            hardwareQueue.add(() -> leftServo.setPosition(transferState.leftServoPos));
            hardwareQueue.add(() -> rightServo.setPosition(transferState.rightServoPos));
            hardwareQueue.add(() -> tiltServo.setPosition(transferState.tiltServoPos));

            update = false;
        }

    }

    public void setTransferState(TransferState transferState) {
        newTransferState = transferState;
    }


}
