package org.firstinspires.ftc.teamcode.util.hardwareQueue;

import java.util.ArrayList;
import java.util.List;

public class VoltageRegulatingHardwareQueue {
    List<HardwareGroup> hardwareGroups = new ArrayList<>();

    List<Device> devices = new ArrayList<>();

    public VoltageRegulatingHardwareQueue() {

    }

    public void addHardwareGroup(HardwareGroup hardwareGroup) {
        hardwareGroups.add(hardwareGroup);
    }

    public void addDevice(Device device) {
        addDevice(device, 1);
    }

    public void addDevice(Device device, int hardwareGroup) {
        device.setHardwareGroup(hardwareGroup);
        device.setId(devices.size());

        devices.add(device);
    }

    public void update() {

    }
}
