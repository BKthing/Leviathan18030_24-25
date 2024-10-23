package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class VoltageRegulatingHardwareQueue {
    private final List<HardwareGroup> hardwareGroups = new ArrayList<>();

    private final List<Device> devices = new ArrayList<>();

    private final VoltageSensor voltageSensor;

    private final double targetOperatingVoltage = 12.0;
    private final double criticalOperatingVoltage = 8.0;

    public VoltageRegulatingHardwareQueue(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void addHardwareGroup(HardwareGroup hardwareGroup) {
        hardwareGroups.add(hardwareGroup);
    }

    public HardwareGroup getHardwareGroup(int hardwareGroup) {
        return hardwareGroups.get(hardwareGroup);
    }

    public void addDevice(Device device) {
        addDevice(device, 1);
    }

    public void addDevice(Device device, int hardwareGroup) {
        device.setId(devices.size());
        devices.add(device);
        hardwareGroups.get(hardwareGroup).addDevice(devices.get(devices.size()-1));
    }

    public Device getDevice(int device) {
        return devices.get(device);
    }

    public ThreadMotor getThreadMotor(int device) {
        if (devices.get(device).getClass() == ThreadMotor.class) {
            return (ThreadMotor) devices.get(device);
        } else {
            throw new RuntimeException("Device is not of type ThreadMotor");
        }
    }

    public void update(HardwareMap hardwareMap) {
        for (Device device : devices) {
            device.updatePriorityVal();
        }
        List<Device> sortedDevices = devices;

        Collections.sort(sortedDevices);


        if (voltageSensor.getVoltage()<targetOperatingVoltage) {

        }
    }
}
