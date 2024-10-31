package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.pathing.EndpointEstimator;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.threading.SubSystemData;

public class EndPointEstimatorSubsystem extends SubSystem {

    private final OldLocalizer localizer;
    private final EndpointEstimator endpointEstimator;

    private Pose2d poseEstimate;// = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity;// = new Pose2d(0 , 0, 0);

    public EndPointEstimatorSubsystem(SubSystemData data, OldLocalizer localizer) {
        super(data);

        this.localizer = localizer;

        endpointEstimator = new EndpointEstimator(RobotConstants.lateralPID, RobotConstants.headingPID, RobotConstants.constraints.getNaturalDecel());
    }

    @Override
    public void priorityData() {
        poseEstimate = localizer.getLocalizer().getPoseEstimate();
        poseVelocity = localizer.getLocalizer().getPoseVelocity();
    }

    @Override
    public void loop() {
        endpointEstimator.updateEndPos(poseEstimate, poseVelocity);

    }

    @Override
    public TelemetryPacket dashboard(TelemetryPacket packet) {
        packet.fieldOverlay().setStroke("#4CAF50");
        DashboardUtil.drawRobot(packet.fieldOverlay(), endpointEstimator.getEstimatedEndPos());

        return packet;
    }
}
