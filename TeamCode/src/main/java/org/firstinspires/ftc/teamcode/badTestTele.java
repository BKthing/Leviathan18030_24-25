package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class badTestTele extends LinearOpMode {
    private DcMotorEx fl;
    private DcMotorEx bl;
    private DcMotorEx fr;
    private DcMotorEx br;

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);

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
                fl.setPower(.5);
            }
            if (gamepad1.b) {
                fr.setPower(0.5);
            }
            if (gamepad1.x){
                br.setPower(.5);
            }
            if (gamepad1.y) {
                bl.setPower(0.5);
            }


            fl.setPower( forward + turn + strafe);
            fr.setPower( forward + turn - strafe);
            bl.setPower( forward + turn - strafe);
            br.setPower( forward - turn + strafe);
        }

    }
}
