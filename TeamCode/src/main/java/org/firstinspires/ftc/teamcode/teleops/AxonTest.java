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
    private Servo leftOuttakeServo;
    private Servo rightOuttakeServo;

    double targetServoPos = 0;

    ElapsedTimer timer = new ElapsedTimer();


    @Override
    public void runOpMode() throws InterruptedException {
        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo");

        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while (!isStopRequested()) {
            leftOuttakeServo.setPosition(targetServoPos);
            rightOuttakeServo.setPosition(targetServoPos);

            if (Math.abs(gamepad1.left_stick_y)>.1) {
                targetServoPos = MathUtil.clip(targetServoPos+gamepad1.left_stick_y*timer.seconds(), 0, 1);
            }

            telemetry.addData("Pos", targetServoPos);

            telemetry.update();

            timer.reset();
        }


    }

}

