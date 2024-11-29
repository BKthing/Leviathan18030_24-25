package org.firstinspires.ftc.teamcode.tuners;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.reefsharklibrary.data.MotorPowers;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.localizers.OldLocalizer;
import com.reefsharklibrary.localizers.OldTwoWheelOldLocalizer;
import com.reefsharklibrary.pathing.EndpointEstimator;
import com.reefsharklibrary.pathing.PIDPointController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import java.util.List;
import java.util.function.DoubleSupplier;

@Autonomous
public class EndPointTesting extends LinearOpMode {


    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in

    public static double PARALLEL_MULTIPLIER = .998342;//.998342

    public static double PERPENDICULAR_MULTIPLIER = 1.01;//1.01

    public static double PARALLEL_Y = -2.57; // Y is the strafe direction 5.8
    public static double PERPENDICULAR_X = -6.49;


    private VoltageSensor batteryVoltageSensor;
    private double voltage = 0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;


    private OldTwoWheelOldLocalizer localizer;

    private EndpointEstimator endpointEstimator;

    private PIDPointController pidPointController;

    private Encoder perpendicularWheel, parallelWheel;

    private DoubleSupplier getPerpendicularWheelDistance, getParallelWheelDistance;

    private MotorPowers motorPowers = new MotorPowers();

    private IMU externalImu;

    private Pose2d targetPoint = new Pose2d(10, 5, Math.toRadians(45));

    private Telemetry.Item error;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");//ex 0
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");//
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");//ex 1

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));//fr
        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalRight"));//bl

        getPerpendicularWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * perpendicularWheel.getCurrentPosition() * PERPENDICULAR_MULTIPLIER / TICKS_PER_REV;
        getParallelWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * parallelWheel.getCurrentPosition() * PARALLEL_MULTIPLIER / TICKS_PER_REV;


        externalImu = hardwareMap.get(BNO055IMUNew.class, "imu");

        externalImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(270, 0, 0))));//new RevHubOrientationOnRobot(xyzOrientation(0, 0, 0)) new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0, 0, 0, 0)
        externalImu.resetYaw();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new OldTwoWheelOldLocalizer(PERPENDICULAR_X, PARALLEL_Y);

        endpointEstimator = new EndpointEstimator(RobotConstants.pointPID, RobotConstants.headingPointPID, RobotConstants.naturalDecel);

        pidPointController = new PIDPointController(RobotConstants.pointPID, RobotConstants.headingPointPID, RobotConstants.trackWidth);


        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        telemetry.setAutoClear(false);

        error = telemetry.addData("Error", "");

        waitForStart();

        localizer.update(getParallelWheelDistance.getAsDouble(), getPerpendicularWheelDistance.getAsDouble(), externalImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));


        while (!isStopRequested()) {
            voltage = batteryVoltageSensor.getVoltage();
            localizer.update(getParallelWheelDistance.getAsDouble(), getPerpendicularWheelDistance.getAsDouble(), externalImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            endpointEstimator.updateEndPos(localizer.getPoseEstimate(), localizer.getPoseVelocity());

//            pidPointController.calculatePowers(localizer.getPoseEstimate(), localizer.getPoseVelocity(), targetPoint, motorPowers, .7);
            pidPointController.calculatePowers(endpointEstimator.getEstimatedEndPos(), endpointEstimator.getEstimatedEndVel(), targetPoint, motorPowers, .7);


            error.setValue(targetPoint.minus(localizer.getPoseEstimate(), Math.PI, -Math.PI));


            setDrivePower(motorPowers);


            motorPowers.reset();
            telemetry.update();
        }
    }


    public void setDrivePower(MotorPowers motorPowers) {
        List<Double> powers = motorPowers.getNormalizedVoltages(voltage);

            frontLeft.setPower(powers.get(0));
            frontRight.setPower(powers.get(3));
            backLeft.setPower(powers.get(1));
            backRight.setPower(powers.get(2));

    }

}
