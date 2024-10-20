package org.firstinspires.ftc.teamcode.teleops;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.teamcode.util.MathUtil;

@TeleOp
public class AxonTest extends LinearOpMode {
    private Servo leftIntakeServo;
    private Servo rightIntakeServo;
    private CRServo intakeServo;
    private double LcurPos, RcurPos;

    double targetServoPos = 0;

    ElapsedTimer timer = new ElapsedTimer();


    @Override
    public void runOpMode() throws InterruptedException {
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        rightIntakeServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
//            if (gamepad2.x) {
//                leftIntakeServo.setPosition(.3);
////                rightIntakeServo.setPosition(.3);
//            }
//            else {
//                leftIntakeServo.setPosition(0);
//            }
//
//            if (gamepad2.a) {
//                intakeServo.setPower(1);
//            }
//            else if (gamepad2.b) {
//                intakeServo.setPower(-1);
//            }
//            else {
//                intakeServo.setPower(0);
//            }

            if (Math.abs(gamepad2.left_stick_y) >0 ) {
                leftIntakeServo.setPosition( leftIntakeServo.getPosition() + .009 + gamepad2.left_stick_y * .00001);
                rightIntakeServo.setPosition(rightIntakeServo.getPosition() + gamepad2.left_stick_y * .00001);

            }



//

            telemetry.addData("Pos", targetServoPos);

            telemetry.update();

            timer.reset();
        }


    }

}

