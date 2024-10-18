package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ThreadMotor implements Device {

    private final String name;
    private int hardwareGroup = 1;
    private int id;

    private final DcMotorEx motor;

    public ThreadMotor(String name, HardwareMap hardwareMap) {
        this.name = name;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean requiresUpdate() {
        return false;
    }

    @Override
    public double getTimeSinceLastUpdate() {
        return 0;
    }

    @Override
    public double getEstimatedAMPS() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public int getHardwareGroup() {
        return hardwareGroup;
    }

    @Override
    public void setHardwareGroup(int hardwareGroup) {
        this.hardwareGroup = hardwareGroup;
    }

    @Override
    public int getId() {
        return 0;
    }

    @Override
    public void setId(int id) {
        this.id = id;
    }
}
