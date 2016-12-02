package org.firstinspires.ftc.teamcode.components;

import android.content.Context;

import java.io.File;
import java.io.FileOutputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;

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

    public static boolean writeStringToFile(Context appContext, String s, String fileName) { //store string in text file in phone's files directory, and return true if successful
        try {
            FileOutputStream o = new FileOutputStream(new File(appContext.getFilesDir(), fileName + ".txt"));
            o.write(s.getBytes());
            o.close();
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {

        }
    }

    public static double average(double a, double b) {
        return (a + b) / 2;
    }

    public static double angleDifference(double a, double b) { //finds signed difference between two angles [-π,π]
        double dif = a - b;
        return dif < -Math.PI ? dif + 2 * Math.PI : dif > Math.PI ? dif - Math.PI * 2 : dif;
    }

    public static ArrayList<Integer[]> findTwoMaxIndexes(double[][] readings) {

        Integer[] maxIndex = findMaxIndex(readings);
        readings[maxIndex[0]][maxIndex[1]] = 0;

        Integer[] secondHighestIndex = findMaxIndex(readings);

        ArrayList<Integer[]> maxes = new ArrayList<Integer[]>();

        maxes.add(maxIndex);
        maxes.add(secondHighestIndex);

        return maxes;
    }

    private static Integer[] findMaxIndex(double[][] readings) {
        Integer[] maxIndex = {0, 0}; // [0, 1]

        for (int i = 0; i < readings.length; i++)
            for (int j = 0; j < readings[i].length; j++)
                if (readings[i][j] > readings[maxIndex[0]][maxIndex[1]])
                    maxIndex = new Integer[] {i, j};

        return maxIndex;
    }

}
