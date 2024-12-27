package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

//@Disabled
@TeleOp
public class BasicBadTele extends LinearOpMode {



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





        waitForStart();

        while ( !isStopRequested()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            fl.setPower(y + x + rx);
            bl.setPower(y - x + rx);
            fr.setPower(y - x - rx);
            br.setPower(y + x - rx);
        }


    }
}




