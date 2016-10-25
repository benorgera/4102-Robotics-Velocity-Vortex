package org.firstinspires.ftc.robotcontroller.internal;

import java.text.DecimalFormat;

/**
 * Created by benorgera on 10/24/16.
 */
public class Utils {

    private static DecimalFormat df = new DecimalFormat("#.##");

    public static double getMagnitude(double x, double y) {
        return Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5);
    }

    public static String toString(double d) { //return double as string with max of two decimal places
        return df.format(d);
    }

    public static double atan3(double y, double x) { //return angle on [0, 2π] given x and y coordinates
        return (Math.atan2(y, x) + 2 * Math.PI) % (Math.PI * 2);
    }
}
