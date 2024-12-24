package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

public class MathUtil {

    public static double clip(double number, double min, double max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

    public static Pose2d toRoadRunnerPose(com.reefsharklibrary.data.Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }

}
