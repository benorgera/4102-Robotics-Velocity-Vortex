package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 1/31/17.
 */

public class GyroPoll implements Runnable {

    private BNO055IMU imu;

    private SizedStack<Double> accelerations = new SizedStack<>(100);

    private final double drivingAccelerationThreshold = 0.5;

    public GyroPoll(BNO055IMU imu) {
        this.imu = imu;
    }

    @Override
    public void run() {
        while (Thread.currentThread().isAlive()) {
            accelerations.push(Math.abs(imu.getAcceleration().xAccel) + Math.abs(imu.getAcceleration().yAccel));
            Hardware.setGyroConstant(getGyroConstant());
            try {
                wait(100);
            } catch (InterruptedException e) {
                return;
            }
        }
    }

    private synchronized double getGyroConstant() {
        return getAverageAcceleration() < drivingAccelerationThreshold ? 1.5 : 1;
    }

    private double getAverageAcceleration() {
        double sum = 0;
        for (Double d : accelerations)
            sum += d;

        return sum / accelerations.size();
    }
}
