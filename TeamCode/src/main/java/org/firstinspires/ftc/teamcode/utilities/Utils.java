package org.firstinspires.ftc.teamcode.utilities;

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

    public static double trim(double min, double max, double val) { //returns val, trimmed to [min, max]
        return val > max ? max : val < min ? min : val;
    }

    public static double getMaxMagnitude(double[] vals) {
        double max = 0;

        for (double a : vals) if (Math.abs(a) > max)
                max = Math.abs(a);

        return max;
    }


    public static double getMaxMagnitude(double[][] vals) { //return the double stored in the arrays with the greatest absolute value
        double max = 0;

        for (double[] d2 : vals) for (double d : d2) {
            d = Math.abs(d);
            if (d > max) max = d;
        }

        return max;
    }

    public static double[][] multiplyValues(double coefficient, double[][] values) { //multiply every value in a 2d array by a number
        for (int i = 0; i < values.length; i++) for (int j = 0; j < values[i].length; j++) //multiply each value by the coefficient
            values[i][j] *= coefficient;

        return values;
    }

    public static double[][] scaleValues(double ceiling, double[][] values) { //divide each value by the maximum magnitude of any of the values, if the maximum magnitude is over the ceiling

        double max = Utils.getMaxMagnitude(values); //maximum magnitude

        if (max <= ceiling) return values; //if none of the values are over the specified ceiling, don't scale anything

        return multiplyValues(1.0 / max, values); //divide each value by the maximum magnitude
    }

    public static double average(double a, double b) {
        return (a + b) / 2;
    }

    public static double angleDifference(double a, double b) { //finds signed difference between two angles [-π,π]
        double dif = a - b;
        return dif < -Math.PI ? dif + 2 * Math.PI : dif > Math.PI ? dif - Math.PI * 2 : dif;
    }

    public static double toDegrees(double rads) {
        return rads * 180 / Math.PI;
    }

}
