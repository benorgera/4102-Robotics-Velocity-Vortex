package org.firstinspires.ftc.robotcontroller.internal;

import java.text.DecimalFormat;

/**
 * Created by benorgera on 10/24/16.
 */
public class Utils {

    private static DecimalFormat df = new DecimalFormat("#.##");

    public static double trim(double val, double min, double max) {
        if (val > max) val = max;
        if (val < min) val = min;
        return val;
    }

    public static double getMagnitude(double x, double y) {
        return Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5);
    }

    public static String toString(double d) { //return double as string with max of two decimal places
        return df.format(d);
    }

}
