package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 2/24/17.
 */

public class PID implements Runnable {

    private DcMotor[] motors;   //motors being controlled by the PID

    private double ngVelDesired;    //our desired motor speed

    private final long dTDesired = 100; //the amount of time we want to sleep for each loop

    private final double kP = 1;    //proportional coefficient
    private final double kI = 0;    //integral coefficient
    private final double kD = 0;    //derivative coefficient

    private double[] motorPowers;   //to store the motor powers
    private double[] integrals;     //the integral of the error throughout running the PID
    private double[] previousErrors;    //the error from the last loop

    private double percentErrorTolerance;   //how close we need to be to our desired speed to consider ourselves done
    private double derivativeTolerance;     //how much change we can have in the error and still consider ourselves at a stable speed (done)

    private boolean[] reachedNgVel = {false, false};

    //initializes the PID
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
            double[] initialPositions = { motors[0].getCurrentPosition(), motors[1].getCurrentPosition() };     //getting the initial encoder values of our motors
            long initialTime = System.currentTimeMillis();  //time at beginning of loop
            Hardware.sleep(dTDesired);  //wait for some change to happen
            for (int i = 0; i < 2; i++)
                motorPowers[i] += getOutput(i, initialPositions[i], previousErrors[i], initialTime);    //add PID output to the motor powers for each motor
        }
    }

    //gives us how much to change the motor power by
    private double getOutput(int index, double initialPosition, double previousError, long initialTime) {
        if (reachedNgVel[index])    //if we are already at the desired speed, return 0 (no change in power)
            return 0;

        long dt = System.currentTimeMillis() - initialTime;     //time elapsed since loop started
        double dTheta = motors[index].getCurrentPosition() - initialPosition,   //change in encoder values
                ngVel = dTheta / dt,    //current speed of motor
                error = this.ngVelDesired - ngVel,  //how much off we are from our desired speed
                proportion = kP * error,    //the adjustment proportional to how much error we have

                //the adjustment proportional to the rate of change in the error. This helps reduce overcompensation
                derivative = kD * (error - previousError) / dt,

                //the adjustment proportional to the sum of all the prior errors. Helps get rid of steady state error
                integral = integrals[index] += kI * error * dt;

        double output = proportion + integral + derivative;     //adjust the motor power by the sum of P I and D

        previousErrors[index] = error;  //store our error as the new previous error

        //if our speed is within a desired percent tolerance of our desired speed,
        //and our derivative is small enough (meaning that we aren't still oscillating a lot
        //we have reached our desired speed an
        if (Math.abs(error) / ngVel < percentErrorTolerance && Math.abs(derivative) < derivativeTolerance) {
            reachedNgVel[index] = true;
            return 0;
        }

        return output;
    }

    //the motors are ready if both of them have reached their desired speeds
    //called by the program running the PID to check if it is done
    public boolean ready() {
        return reachedNgVel[0] && reachedNgVel[1];
    }

    //resets encoders and reachedNgVel
    public void reset() {
        reachedNgVel = new boolean[] {false, false};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
