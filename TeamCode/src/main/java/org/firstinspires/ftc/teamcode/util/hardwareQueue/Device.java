package org.firstinspires.ftc.teamcode.util.hardwareQueue;

public interface Device {

    void update();
    boolean requiresUpdate();
    double getTimeSinceLastUpdate();

    double getEstimatedAMPS();

    String getDeviceName();

    int getHardwareGroup();

    void setHardwareGroup(int hardwareGroup);

    int getId();

    void setId(int id);


}
