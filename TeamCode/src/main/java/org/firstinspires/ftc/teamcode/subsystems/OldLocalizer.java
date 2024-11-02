package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.localizers.OldTwoWheelOldLocalizer;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.function.DoubleSupplier;

public class OldLocalizer extends SubSystem{
    OldTwoWheelOldLocalizer localizer;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in

    public static double PARALLEL_MULTIPLIER = .998342;//.998342

    public static double PERPENDICULAR_MULTIPLIER = 1.01;//1.01

    public static double PARALLEL_Y = -2.56890; // Y is the strafe direction 5.8
    public static double PERPENDICULAR_X = -5.98040;//2.246

    private final Encoder perpendicularWheel, parallelWheel;

    private final IMU externalImu, expansionImu, controlImu;


    private enum CurrentImu{
        EXTERNAL_IMU,
        EXPANSION_IMU,
        CONTROL_IMU;
    }

    CurrentImu currentImu = CurrentImu.EXTERNAL_IMU;


    private double imuAngle = 0;
    private double updatedImuAngle = 0;

    private double imuOffsetAngle = 0;

    private final DoubleSupplier getPerpendicularWheelDistance, getParallelWheelDistance;

    private double perpendicularWheelDistance, parallelWheelDistance;

    private final Telemetry.Item position;
    private final Telemetry.Item imuType;
//    private final Telemetry.Item rawPosition;
//    private final Telemetry.Item velocity;

    private ElapsedTimer timer = new ElapsedTimer();

    public OldLocalizer(SubSystemData data) {
        super(data);

        //initializing localizer and sensors
        position = data.getTelemetry().addData("Pose Estimate (old)", new Pose2d(0, 0 ,0));
        imuType = data.getTelemetry().addData("IMU", currentImu);
//        rawPosition = data.getTelemetry().addData("Raw Pos", "");
//        velocity = data.getTelemetry().addData("Velocity", new Pose2d(0, 0, 0));


        localizer = new OldTwoWheelOldLocalizer(PERPENDICULAR_X, PARALLEL_Y);

        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalRight"));//fr
        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));//bl


        getPerpendicularWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * perpendicularWheel.getCurrentPosition() * PERPENDICULAR_MULTIPLIER / TICKS_PER_REV;
        getParallelWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * parallelWheel.getCurrentPosition() * PARALLEL_MULTIPLIER / TICKS_PER_REV;

        externalImu = hardwareMap.get(BNO055IMUNew.class, "imu");
        expansionImu = hardwareMap.get(BNO055IMUNew.class, "expansionImu");
        controlImu = hardwareMap.get(BHI260IMU.class, "controlImu");

        externalImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(270, 0, 0))));//new RevHubOrientationOnRobot(xyzOrientation(0, 0, 0)) new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0, 0, 0, 0)
        externalImu.resetYaw();

        expansionImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        controlImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

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

        if (Double.isNaN(updatedImuAngle)) {
            changeImu();
        } else {
            imuAngle = updatedImuAngle + imuOffsetAngle;
        }


        }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        localizer.update(parallelWheelDistance, perpendicularWheelDistance, imuAngle);

        //request an imu call, only gets called if it is not already queued
//        imuAction.queueAction();
        switch (currentImu) {
            case EXTERNAL_IMU:
                data.getHardwareQueue().add(() -> {
                    updatedImuAngle = externalImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
                });
                break;
            case EXPANSION_IMU:
                data.getHardwareQueue().add(() -> {
                    updatedImuAngle = expansionImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
                });
                break;
            case CONTROL_IMU:
                data.getHardwareQueue().add(() -> {
                    updatedImuAngle = controlImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
                });
                break;
        }

    }

    public void update() {
        localizer.update(getPerpendicularWheelDistance.getAsDouble(), getParallelWheelDistance.getAsDouble(), externalImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
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

    private void changeImu() {
            if (currentImu != CurrentImu.EXTERNAL_IMU) {
                double testIMUAngle = externalImu.getRobotYawPitchRollAngles().getYaw();
                if (!Double.isNaN(testIMUAngle)) {
                    currentImu = CurrentImu.EXTERNAL_IMU;
                    //imuAngle = testImuangle + offset
                    imuOffsetAngle = imuAngle-testIMUAngle;
                    imuType.setValue(currentImu);
                    return;
                }
            }

            if (currentImu != CurrentImu.EXPANSION_IMU) {
                double testIMUAngle = expansionImu.getRobotYawPitchRollAngles().getYaw();
                if (!Double.isNaN(testIMUAngle)) {
                    currentImu = CurrentImu.EXPANSION_IMU;

                    imuOffsetAngle = imuAngle-testIMUAngle;
                    imuType.setValue(currentImu);
                    return;
                }
            }

            if (currentImu != CurrentImu.CONTROL_IMU) {
                double testIMUAngle = controlImu.getRobotYawPitchRollAngles().getYaw();
                if (!Double.isNaN(testIMUAngle)) {
                    currentImu = CurrentImu.CONTROL_IMU;

                    imuOffsetAngle = imuAngle-testIMUAngle;
                    imuType.setValue(currentImu);
                    return;
                }
            }
    }
}
