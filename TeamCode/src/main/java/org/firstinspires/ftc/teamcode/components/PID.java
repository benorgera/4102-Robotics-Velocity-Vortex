package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 2/24/17.
 */

public class PID implements Runnable {

    private DcMotor[] motors;

    private double ngVelDesired;

    private final long dTDesired = 100;

    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;

    private double[] motorPowers;
    private double[] integrals;
    private double[] previousErrors;

    private double percentErrorTolerance;
    private double derivativeTolerance;

    private boolean[] reachedNgVel = {false, false};

    public PID(DcMotor[] motors, double ngVelDesired, double initialPower, double percentErrorTolerance, double derivativeTolerance) {
        this.motors = motors;
        this.ngVelDesired = ngVelDesired;

        this.motorPowers = new double[] {initialPower, initialPower};
        this.integrals = new double[] {0, 0};
        this.previousErrors = new double[] {0, 0};

        this.derivativeTolerance = derivativeTolerance;
        this.percentErrorTolerance = percentErrorTolerance;
    }


    public void run() {
        while (Hardware.active() && !Thread.currentThread().isInterrupted() && Thread.currentThread().isAlive()) {
            double[] initialPositions = { motors[0].getCurrentPosition(), motors[1].getCurrentPosition() };
            long initialTime = System.currentTimeMillis();
            Hardware.sleep(dTDesired);
            for (int i = 0; i < 2; i++)
                motorPowers[i] += getOutput(i, initialPositions[i], previousErrors[i], initialTime);
        }
    }

    private double getOutput(int index, double initialPosition, double previousError, long initialTime) {
        if (reachedNgVel[index])
            return 0;

        long dt = System.currentTimeMillis() - initialTime;
        double dTheta = motors[index].getCurrentPosition() - initialPosition,
                ngVel = dTheta / dt,
                error = this.ngVelDesired - ngVel,
                proportion = kP * error,
                derivative = kD * (error - previousError) / dt,
                integral = integrals[index] += kI * error * dt;

        double output = proportion + integral + derivative;

        previousErrors[index] = error;

        if (Math.abs(error) / ngVel < percentErrorTolerance && Math.abs(derivative) < derivativeTolerance) {
            reachedNgVel[index] = true;
            return 0;
        }

        return output;
    }

    public boolean ready() {
        return reachedNgVel[0] && reachedNgVel[1];
    }

    public void reset() {
        reachedNgVel = new boolean[] {false, false};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
