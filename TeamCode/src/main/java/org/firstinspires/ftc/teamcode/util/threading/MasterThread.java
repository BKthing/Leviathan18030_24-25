package org.firstinspires.ftc.teamcode.util.threading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.reefsharklibrary.misc.ElapsedTimer;
import com.reefsharklibrary.robotControl.HardwareQueue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class MasterThread {

    private final HardwareQueue hardwareQueue = new HardwareQueue();

    private final List<SubSystem> subSystems = new ArrayList<>();

    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private final Gamepad gamepad1Instance = new Gamepad();
    private final Gamepad gamepad2Instance = new Gamepad();

    private final FtcDashboard dashboard;

    private final boolean dashboardEnabled = true;

    private final SubSystemData data;

    private final List<LynxModule> allHubs;

    private final Telemetry.Item queueSize;

    private final Telemetry.Item threadUpdateTime;
    private final ElapsedTimer threadUpdateTimer = new ElapsedTimer();

    public MasterThread(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;

        dashboard = FtcDashboard.getInstance();

        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()) ;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        queueSize = telemetry.addData("Queue Size", 0);

        threadUpdateTime = telemetry.addData("Thread update time", "");

        this.data = new SubSystemData(hardwareMap, hardwareQueue, telemetry, gamepad1Instance, gamepad2Instance);

        allHubs = hardwareMap.getAll(LynxModule.class);
        manualBulkReads(true);
    }

    public void addSubSystem(SubSystem subSystem) {
        subSystems.add(subSystem);
    }

    public void addSubSystems(SubSystem... subSystem) {
        subSystems.addAll(Arrays.asList(subSystem));
    }

    public void unThreadedUpdate() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        //passes an instance so that all subsystems will run with the same set of buttons pressed
        gamepad1Instance.copy(gamepad1);
        gamepad2Instance.copy(gamepad2);


//        ExecutorService es = Executors.newCachedThreadPool();
        for (SubSystem subSystem: subSystems) {
            subSystem.priorityData();
            subSystem.loop();
        }

//        es.shutdown();

        threadUpdateTimer.reset();

        //updates the data in the bulkCache
        clearBulkCache();


        //ensures that a certain amount of hardware actions are called
        int minHardwareUpdates = 8;

        //runs queued actions while threads are still active
//        while (!es.awaitTermination(1, TimeUnit.NANOSECONDS)) {
//            if (hardwareQueue.updateSingle()) {
//                minHardwareUpdates -= 1;
//            }
//        }

        hardwareQueue.update(minHardwareUpdates, 5);

        queueSize.setValue(hardwareQueue.size());

        if (dashboardEnabled) {
            for (SubSystem subSystem: subSystems) {
                packet = subSystem.dashboard(packet);
            }
        }

        dashboard.sendTelemetryPacket(packet);
        dashboard.getTelemetry().update();

        telemetry.update();
    }

    public void update() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        //passes an instance so that all subsystems will run with the same set of buttons pressed
        gamepad1Instance.copy(gamepad1);
        gamepad2Instance.copy(gamepad2);


        ExecutorService es = Executors.newCachedThreadPool();
        for (SubSystem subSystem: subSystems) {
            subSystem.update(es);
        }

        es.shutdown();

        //updates the data in the bulkCache
        clearBulkCache();


        //ensures that a certain amount of hardware actions are called
        int minHardwareUpdates = 8;

        //runs queued actions while threads are still active
        while (!es.awaitTermination(1, TimeUnit.NANOSECONDS)) {
            if (hardwareQueue.updateSingle()) {
                minHardwareUpdates -= 1;
            }
        }

        hardwareQueue.update(minHardwareUpdates, 5);

        queueSize.setValue(hardwareQueue.size());

        if (dashboardEnabled) {
            for (SubSystem subSystem: subSystems) {
                packet = subSystem.dashboard(packet);
            }
        }

        dashboard.sendTelemetryPacket(packet);
        dashboard.getTelemetry().update();

        threadUpdateTime.setValue(threadUpdateTimer.milliSeconds());

        telemetry.update();

    }

    public void manualBulkReads(boolean manualReads) {
        if (manualReads) {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } else {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public SubSystemData getData() {
        return data;
    }
}
