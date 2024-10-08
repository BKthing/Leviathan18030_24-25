package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class badTestTele extends LinearOpMode {
    private DcMotorEx fl;
    private DcMotorEx bl;
    private DcMotorEx fr;
    private DcMotorEx br;

    private DcMotorEx horizontalLeftMotor;
    private DcMotorEx horizontalRightMotor;

    private DcMotorEx verticalLeftMotor;
    private DcMotorEx verticalRightMotor;

    private Encoder verticalSlideEncoder;
    private Encoder horizontalSlideEncoder;

    private Servo leftServo, rightServo, tiltServo;

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLeftMotor = hardwareMap.get(DcMotorEx.class, "horizontalLeft"); // control hub 0
        horizontalRightMotor = hardwareMap.get(DcMotorEx.class, "horizontalRight"); // control hub 1

        horizontalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeftMotor = hardwareMap.get(DcMotorEx.class, "verticalLeft"); // control hub 2
        verticalRightMotor = hardwareMap.get(DcMotorEx.class, "verticalRight"); // control hub 3

//        verticalRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));
        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));

        horizontalSlideEncoder.setDirection(Encoder.Direction.REVERSE);
        verticalSlideEncoder.setDirection(Encoder.Direction.REVERSE);

        Telemetry.Item horizontalEncoderPos = telemetry.addData("Horizontal Ticks", "");
        Telemetry.Item verticalEncoderPos = telemetry.addData("Vertical Ticks", "");


        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");



        telemetry.setAutoClear(false);



        waitForStart();

        while ( !isStopRequested()) {

            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if ( Math.abs(strafe) < .02)
                strafe = 0;

            if ( Math.abs(forward) < .02)
                forward = 0;

            if ( Math.abs(turn) < .02)
                turn = 0;

            if (gamepad1.a){
                horizontalLeftMotor.setPower(.5);
                horizontalRightMotor.setPower(0.5);
            } else if (gamepad1.b) {
                horizontalLeftMotor.setPower(-.5);
                horizontalRightMotor.setPower(-0.5);
            } else {
                horizontalLeftMotor.setPower(0);
                horizontalRightMotor.setPower(0);
            }

            if (gamepad1.x){
                verticalLeftMotor.setPower(.5);
                verticalRightMotor.setPower(0.5);
            } else if (gamepad1.y) {
                verticalLeftMotor.setPower(-.5);
                verticalRightMotor.setPower(-0.5);
            } else {
                verticalLeftMotor.setPower(0);
                verticalRightMotor.setPower(0);
            }

            leftServo.setPosition(.46);
            rightServo.setPosition(.46);
//            if (gamepad1.b) {
//                horizontalRightMotor.setPower(0.5);
//            } else  {
//                horizontalRightMotor.setPower(0);
//            }
//            if (gamepad1.x){
//                br.setPower(.5);
//            }
//            if (gamepad1.y) {
//                bl.setPower(0.5);
//            }


            fl.setPower( forward + turn + strafe);
            fr.setPower( forward - turn - strafe);
            bl.setPower( forward + turn - strafe);
            br.setPower( forward - turn + strafe);

            horizontalEncoderPos.setValue(horizontalSlideEncoder.getCurrentPosition());
            verticalEncoderPos.setValue(verticalSlideEncoder.getCurrentPosition());
            telemetry.update();
        }

    }
}
