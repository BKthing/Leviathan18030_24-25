package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    public static double clip(double number, double min, double max) {
        if (number < min) return min;
        if (number > max) return max;
        return number;
    }

}
