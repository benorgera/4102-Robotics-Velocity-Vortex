package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 1/31/17.
 */

public class GyroPoll implements Runnable {

    private BNO055IMU imu;

    private SizedStack<Double> accelerations = new SizedStack<Double>(100);

    private final double drivingAccelerationThreshold = 0.650;

    public GyroPoll(BNO055IMU imu) {
        this.imu = imu;
    }

    @Override
    public void run() {

        try {
            while (!Thread.currentThread().isInterrupted()) {
                accelerations.push(Math.sqrt(Math.pow(imu.getLinearAcceleration().xAccel, 2) + Math.pow(imu.getLinearAcceleration().yAccel, 2)));
                Hardware.setGyroConstant(getGyroConstant());
                Thread.sleep(40);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private double getGyroConstant() {
        Hardware.clearLog();
        Hardware.print("avg a: " + getAverageAcceleration());

        double gyroConst = 1;

        if (accelerations.size() > 50 && getAverageAcceleration() < drivingAccelerationThreshold)
            gyroConst = Math.sin(2 * Math.PI / 1000 * System.currentTimeMillis()) > -0.5 ? 1.5 : -0.5;

        Hardware.print("gConst:" + gyroConst);
        return gyroConst;
    }

    private double getAverageAcceleration() {
        double sum = 0;
        for (Double d : accelerations)
            sum += d;

        return sum / accelerations.size();
    }
}
