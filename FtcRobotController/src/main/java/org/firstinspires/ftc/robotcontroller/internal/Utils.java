package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Locale;

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

    public static double trim(double min, double max, double val) { //returns val, trimmed to [min, max]
        return val > max ? max : val < min ? min : val;
    }

    public static double getMaxMagnitude(double[] vals) {
        double max = 0;

        for (double d : vals) {
            d = Math.abs(d);
            if (d > max) max = d;
        }

        return max;
    }

    public static double getMaxMagnitude(double[][] vals) {
        double max = 0;

        for (double[] d2 : vals) for (double d : d2) {
            d = Math.abs(d);
            if (d > max) max = d;
        }

        return max;
    }


}
