package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.localizers.OldTwoWheelOldLocalizer;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.function.DoubleSupplier;

public class OldLocalizer extends SubSystem{
    OldTwoWheelOldLocalizer localizer;
    ReusableHardwareAction imuAction;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in

    public static double PARALLEL_MULTIPLIER = 1.01;//.998342

    public static double PERPENDICULAR_MULTIPLIER = -.998342;//1.01

    public static double PARALLEL_Y = 1.8799213; // Y is the strafe direction 5.8
    public static double PERPENDICULAR_X = 2.9429134;//2.246

    private final Encoder perpendicularWheel, parallelWheel;

    private final IMU imu;

    private double imuAngle = 0;
    private double updatedImuAngle = 0;

    private final DoubleSupplier getPerpendicularWheelDistance, getParallelWheelDistance;

    private double perpendicularWheelDistance, parallelWheelDistance;

    private final Telemetry.Item position;
//    private final Telemetry.Item rawPosition;
//    private final Telemetry.Item velocity;

    private ElapsedTimer timer = new ElapsedTimer();

    public OldLocalizer(SubSystemData data) {
        super(data);

        //initializing localizer and sensors
        position = data.getTelemetry().addData("Pose Estimate (old)", new Pose2d(0, 0 ,0));
//        rawPosition = data.getTelemetry().addData("Raw Pos", "");
//        velocity = data.getTelemetry().addData("Velocity", new Pose2d(0, 0, 0));


        localizer = new OldTwoWheelOldLocalizer(PERPENDICULAR_X, PARALLEL_Y);

        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));
        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));


        getPerpendicularWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * perpendicularWheel.getCurrentPosition() * PERPENDICULAR_MULTIPLIER / TICKS_PER_REV;
        getParallelWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * parallelWheel.getCurrentPosition() * PARALLEL_MULTIPLIER / TICKS_PER_REV;

        imuAction = new ReusableHardwareAction(data.getHardwareQueue());

        imu = hardwareMap.get(BNO055IMUNew.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(270, 0, 0))));//new RevHubOrientationOnRobot(xyzOrientation(0, 0, 0)) new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0, 0, 0, 0)
        imu.resetYaw();

//        imuAction.setAction(() -> {
//            imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)*Math.PI/180;
//            position.setValue("imu updated");
//        });

    }

    @Override
    public void priorityData() {
        perpendicularWheelDistance = getPerpendicularWheelDistance.getAsDouble();
        parallelWheelDistance = getParallelWheelDistance.getAsDouble();
        //is set here to prevent threading conflicts
        imuAngle = updatedImuAngle;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        localizer.update(parallelWheelDistance, perpendicularWheelDistance, imuAngle);

        //request an imu call, only gets called if it is not already queued
//        imuAction.queueAction();
        data.getHardwareQueue().add(() -> {
            updatedImuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
        });
    }

    public void update() {
        localizer.update(getPerpendicularWheelDistance.getAsDouble(), getParallelWheelDistance.getAsDouble(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        position.setValue(localizer.getPoseEstimate().toString());
//        rawPosition.setValue("Perp: " + perpendicularWheelDistance + " Parallel: " + parallelWheelDistance);

        packet.put("x (old)", localizer.getPoseEstimate().getX());
        packet.put("y (old)", localizer.getPoseEstimate().getY());
        packet.put("heading (deg) (old)", localizer.getPoseEstimate().getHeading()*180/Math.PI);

        packet.fieldOverlay().setStrokeWidth(1);

        //draws the last 200 points the robot was at
        packet.fieldOverlay().setStroke("#3F51B5");
        DashboardUtil.drawFullPoseHistory(packet.fieldOverlay(), localizer.getPoseHistory());

        //draws the robots current position
        DashboardUtil.drawRobot(packet.fieldOverlay(), localizer.getPoseEstimate());

        Vector2d robotVelocity = localizer.getPoseVelocity().getVector2d();

//        velocity.setValue(robotVelocity);

        DashboardUtil.drawArrow(packet.fieldOverlay(), localizer.getPoseEstimate().getVector2d(), localizer.getPoseEstimate().getVector2d().plus(robotVelocity));

        return packet;
    }

    public com.reefsharklibrary.localizers.OldLocalizer getLocalizer() {
        return localizer;
    }
}
