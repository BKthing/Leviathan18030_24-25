package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.reefsharklibrary.pathing.data.MarkerExecutable;

public class SingleAction implements Action {
    private final MarkerExecutable executable;

    public SingleAction(MarkerExecutable executable) {
        this.executable = executable;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        executable.run();
        return false;
    }
}
