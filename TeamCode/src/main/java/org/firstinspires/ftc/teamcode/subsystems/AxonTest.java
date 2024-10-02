package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

@TeleOp
public class AxonTest extends LinearOpMode {
    private Servo servo;




    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "axonMicro");

        waitForStart();
        while ( !isStopRequested()){
            if (gamepad1.left_bumper) {
                servo.setPosition(.5);
            }
            else
                servo.setPosition(0);
            }
        }


    }

