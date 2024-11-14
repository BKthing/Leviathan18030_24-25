package org.firstinspires.ftc.teamcode.teleops;

import android.graphics.Color;
import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.reefsharklibrary.misc.ElapsedTimer;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@TeleOp
public class AxonTest extends LinearOpMode {
    private Servo leftIntakeServo, rightIntakeServo, claw, hangDeploy, leftOuttake, rightOuttake, clawWristRoll, clawWristPitch;
    private CRServo intakeServo;
    private double LcurPos, RcurPos;
    private CRServo leftIntake, rightIntake;
    private NormalizedColorSensor colorSensor;
    Telemetry.Item colorTelem;
    Telemetry.Item sampleColorTelem;
    Telemetry.Item loopTime;



    ElapsedTimer timer = new ElapsedTimer();


    @Override
    public void runOpMode() throws InterruptedException {
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        colorTelem = telemetry.addData("Color RGB", "");
        sampleColorTelem = telemetry.addData("Sample Color:", "");
        loopTime = telemetry.addData("Loop Time:", "");


//        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
//        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");


//        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
//        claw = hardwareMap.get(Servo.class, "clawServo");
//        hangDeploy = hardwareMap.get(Servo.class, "hangDeploy");
//        leftOuttake = hardwareMap.get(Servo.class, "leftOuttakeServo");
//        rightOuttake = hardwareMap.get(Servo.class, "rightOuttakeServo");
//        clawWristRoll = hardwareMap.get(Servo.class, "wristRollServo");
//        clawWristPitch = hardwareMap.get(Servo.class, "wristPitchServo");


//        rightOuttake.setDirection(Servo.Direction.REVERSE);
//        leftOuttake.scaleRange(.34, .965);
//        rightOuttake.scaleRange(1-.965, 1-.34);
//
//        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
//        leftIntakeServo.scaleRange(.34, .965);
//        rightIntakeServo.scaleRange(.34, .965);
//
//        clawWristRoll.scaleRange(.34, .965);
//        clawWristPitch.scaleRange(.34, .965);


//        double targetPos = .2;
//
//        double clawRollPos = .3;

        waitForStart();
        while (!isStopRequested()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            if (gamepad2.a) {
                if (colors.red > .006 && colors.green < .01) {
                    sampleColorTelem.setValue("red");
                    leftIntake.setPower(.8);
                    rightIntake.setPower(.8);
                }
                else {
                    leftIntake.setPower(-.8);
                    rightIntake.setPower(-.8);
                }

            }
            else if (gamepad2.b) {
                leftIntake.setPower(.8);
                rightIntake.setPower(.8);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            colorTelem.setValue(colors.red + " " + colors.green + " " + colors.blue);

            if (colors.red > .006 && colors.green < .01) {
                sampleColorTelem.setValue("red");
            }
            else if (colors.blue > .006) {
                sampleColorTelem.setValue("blue");
            }
            else if (colors.green > .01) {
                sampleColorTelem.setValue("yellow");
            }
            else {
                sampleColorTelem.setValue("no sample");
            }

            loopTime.setValue(timer);
//            if (colorSensor.argb() == Color.rgb(0, 0, 0)){
//
//            }
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

//            if (gamepad2.a) {
//                claw.setPosition(.35); //open
//            }
//
//            else if (gamepad2.b) {
//                claw.setPosition(.1); //close
//            }
//
//            if (gamepad2.x) {
//                hangDeploy.setPosition(.4); // deploy
//            }
//           else  if (gamepad2.y) {
//                hangDeploy.setPosition(.2); // not deploy
//            }

//           if (Math.abs (gamepad2.left_stick_y) > .1 ) {
////               clawWristPitch.setPosition(targetPos);
////               leftIntakeServo.setPosition( targetPos + .006);
////               rightIntakeServo.setPosition(targetPos);
//               leftOuttake.setPosition( targetPos);
//               rightOuttake.setPosition(targetPos);
//               targetPos = MathUtil.clip(targetPos + gamepad2.left_stick_y * .0002 * timer.milliSeconds(), 0, 1);
//           }
//
//           if (Math.abs (gamepad2.right_stick_x) > .1) {
//               clawWristRoll.setPosition(clawRollPos);
//               clawRollPos = MathUtil.clip(clawRollPos + gamepad2.right_stick_x * .0002 * timer.milliSeconds(), 0, 1);
//           }




//            leftIntakeServo.setPosition( targetPos + .006);
//            rightIntakeServo.setPosition(targetPos);



//

//            telemetry.addData("Pos", targetPos);
//            telemetry.addData("Roll Pos", clawRollPos);


            telemetry.update();

            timer.reset();
        }


    }

}

