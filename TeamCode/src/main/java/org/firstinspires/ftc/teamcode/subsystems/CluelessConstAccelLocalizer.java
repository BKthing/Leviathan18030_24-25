package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.reefsharklibrary.data.Point;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.Vector2d;
import com.reefsharklibrary.localizers.CluelessConstantAccelMath;
import com.reefsharklibrary.localizers.CluelessTwoWheelLocalizer;
import com.reefsharklibrary.localizers.ConstantAccelSolver;
import com.reefsharklibrary.localizers.Localizer;
import com.reefsharklibrary.localizers.TwoWheel;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.ReusableHardwareAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

import java.util.function.DoubleSupplier;

public class CluelessConstAccelLocalizer extends SubSystem{
    CluelessTwoWheelLocalizer localizer;
//    ReusableHardwareAction imuAction;

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.94; // in

    public static double PARALLEL_MULTIPLIER = -1;//.998342

    public static double PERPENDICULAR_MULTIPLIER = -1;//1.01

    public static double PARALLEL_Y = -3.08; //2.57
    public static double PERPENDICULAR_X = -4.75;//6.49

    private Encoder perpendicularWheel, parallelWheel;

    private final IMU imu;

    private double imuAngle = 0;
    private double updatedImuAngle = 0;

    private final DoubleSupplier getPerpendicularWheelDistance, getParallelWheelDistance;

    private double perpendicularWheelDistance, parallelWheelDistance;

    private final Telemetry.Item position;
//    private final Telemetry.Item rawPosition;
    private final Telemetry.Item velocity;

    private final ElapsedTimer timer = new ElapsedTimer();

//    private final Telemetry.Item localizerLoopTime;

//    private final ElapsedTimer localizerLoopTimer = new ElapsedTimer();

//    TwoWheel twoWheel;

    public CluelessConstAccelLocalizer(SubSystemData data) {
        super(data);

        //initializing localizer and sensors
        position = telemetry.addData("Clueless Pos", new Pose2d(0, 0 ,0));
//        rawPosition = telemetry.addData("Raw Pos", "");
        velocity = telemetry.addData("Velocity", new Pose2d(0, 0, 0));

        localizer = new CluelessTwoWheelLocalizer(PERPENDICULAR_X, PARALLEL_Y);

//        perpendicularWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalRight"));
//        parallelWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "bl"));

        getPerpendicularWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * perpendicularWheel.getCurrentPosition() * PERPENDICULAR_MULTIPLIER / TICKS_PER_REV;
        getParallelWheelDistance = () -> WHEEL_RADIUS * 2 * Math.PI * parallelWheel.getCurrentPosition() * PARALLEL_MULTIPLIER / TICKS_PER_REV;

//        imuAction = new ReusableHardwareAction(data.getHardwareQueue());

//        imu = hardwareMap.get(BNO055IMUNew.class, "imu");

//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(270, 0, 0))));//new RevHubOrientationOnRobot(xyzOrientation(0, 0, 0)) new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0, 0, 0, 0)
//        imu.resetYaw();
        imu = hardwareMap.get(BNO055IMUNew.class, "expansionImu");


        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));


//        localizerLoopTime = telemetry.addData("Localizer loop time", "");
//        twoWheel = new TwoWheel(PERPENDICULAR_X, PARALLEL_Y);

//        imuAction.setAction(() -> {
//            imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)*Math.PI/180;
//            position.setValue("imu updated");
//        });`

    }

    public void initSensors(Encoder perpendicularWheel, Encoder parallelWheel) {
        this.perpendicularWheel = perpendicularWheel;
        this.parallelWheel = parallelWheel;
    }

    @Override
    public void priorityData() {
//        localizerLoopTimer.reset();
        perpendicularWheelDistance = getPerpendicularWheelDistance.getAsDouble();
        parallelWheelDistance = getParallelWheelDistance.getAsDouble();
        //is set here to prevent threading conflicts
        imuAngle = updatedImuAngle;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        localizer.update(parallelWheelDistance, perpendicularWheelDistance, imuAngle, timer.seconds());
//        twoWheel.update(new Point(parallelWheelDistance, timer.seconds()), new Point(perpendicularWheelDistance, timer.seconds()), new Point(imuAngle, timer.seconds()));
        //request an imu call, only gets called if it is not already queued
//        imuAction.queueAction();
        data.getHardwareQueue().add(() -> {
            updatedImuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
        });

//        localizerLoopTime.setValue(localizerLoopTimer.milliSeconds());
    }

    public void update() {
//        localizer.update(parallelWheelDistance, perpendicularWheelDistance, imuAngle, timer.nanoSeconds());
//        localizer.update(0, 0, 0, 1);
    }

    public void clearDeltas() {
        updatedImuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//*Math.PI/180
        imuAngle = updatedImuAngle;

        perpendicularWheelDistance = getPerpendicularWheelDistance.getAsDouble();
        parallelWheelDistance = getParallelWheelDistance.getAsDouble();

        localizer.clearDeltas(parallelWheelDistance, perpendicularWheelDistance, imuAngle);
    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        position.setValue(localizer.getPoseEstimate());//((Pose2d) localizer.getPoseEstimate()).toString()

        packet.put("x", localizer.getPoseEstimate().getX());
        packet.put("y", localizer.getPoseEstimate().getY());
        packet.put("heading (deg)", localizer.getPoseEstimate().getHeading()*180/Math.PI);

        packet.fieldOverlay().setStrokeWidth(1);
        packet.fieldOverlay().setStroke("#b33fb5");
        DashboardUtil.drawFullPoseHistory(packet.fieldOverlay(), localizer.getPoseHistory());
        DashboardUtil.drawRobot(packet.fieldOverlay(), localizer.getPoseEstimate());

        //draws the last 200 points the robot was at

        packet.fieldOverlay().setStroke("#3F51B5");


        //draws the robots current position

        Vector2d robotVelocity = localizer.getPoseVelocity().getVector2d();

        velocity.setValue(localizer.getPoseVelocity());

        DashboardUtil.drawArrow(packet.fieldOverlay(), localizer.getPoseEstimate().getVector2d(), localizer.getPoseEstimate().getVector2d().plus(robotVelocity));

        return packet;
    }

    public CluelessTwoWheelLocalizer getLocalizer() {
        return localizer;
    }

//    public TwoWheel getTwoWheel() {return twoWheel;}
}
