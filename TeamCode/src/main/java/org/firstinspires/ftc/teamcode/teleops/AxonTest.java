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
    private Servo leftIntakeServo, rightIntakeServo, claw, hangDeploy, leftOuttake, rightOuttake, clawWristRoll;
    private CRServo intakeServo;
    private double LcurPos, RcurPos;


    ElapsedTimer timer = new ElapsedTimer();


    @Override
    public void runOpMode() throws InterruptedException {
//        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
//        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
//        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        claw = hardwareMap.get(Servo.class, "clawServo");
        hangDeploy = hardwareMap.get(Servo.class, "hangDeploy");
        leftOuttake = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttake = hardwareMap.get(Servo.class, "rightOuttakeServo");
        clawWristRoll = hardwareMap.get(Servo.class, "wristRollServo");

        rightOuttake.setDirection(Servo.Direction.REVERSE);
//        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
        leftOuttake.scaleRange(.34, .965);
        rightOuttake.scaleRange(1-.965, 1-.34);

        clawWristRoll.scaleRange(.34, .965);

        double targetPos = .8;

        double clawRollPos = 0;

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

            if (gamepad2.a) {
                claw.setPosition(.35); //open
            }

            else if (gamepad2.b) {
                claw.setPosition(.1); //close
            }

            if (gamepad2.x) {
                hangDeploy.setPosition(.4); // deploy
            }
           else  if (gamepad2.y) {
                hangDeploy.setPosition(.2); // not deploy
            }

           if (Math.abs (gamepad2.left_stick_y) > .1 ) {
               leftOuttake.setPosition( targetPos);
               rightOuttake.setPosition(targetPos);
               targetPos = MathUtil.clip(targetPos + gamepad2.left_stick_y * .001 * timer.milliSeconds(), 0, 1);
           }

           if (Math.abs (gamepad2.right_stick_x) > .1) {
               clawWristRoll.setPosition(clawRollPos);
               clawRollPos = MathUtil.clip(clawRollPos + gamepad2.right_stick_x * .0001 * timer.milliSeconds(), 0, 1);
           }




//            leftIntakeServo.setPosition( targetPos + .006);
//            rightIntakeServo.setPosition(targetPos);



//

            telemetry.addData("Pos", targetPos);

            telemetry.update();

            timer.reset();
        }


    }

}

