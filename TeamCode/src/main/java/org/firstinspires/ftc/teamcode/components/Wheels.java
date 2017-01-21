package org.firstinspires.ftc.teamcode.components;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.components.Utils;

/**
 * Created by benorgera on 10/24/16.
 */
public class Wheels {
    private DcMotor[][] wheelbase;

    //constants
    private final double channelModeStickThreshold = 0.1; //the max magnitude of a stick reading
    private final double significantMovementThreshold = 0.05; //the wheel power required to be considered significant movement
    private final double softStopPercentage = 0.8;

    private final double softStartIterations = 10;

    private final long softStopSleepTime = 100;

    private final double[][] compensationConstants = {
            {1, 1},
            {1, 1}
    };

    public Wheels(DcMotor[][] wheelBase) {

        this.wheelbase = wheelBase; //initialize wheel motors

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) { //set up motors
            wheelbase[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheelbase[i][j].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //reverse the motors which face opposite directions (the right motors)
        wheelbase[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBase[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void softStart(double xVel, double yVel, double angularVel) {
        xVel /= 100;
        yVel /= 100;
        angularVel /= 100;

        for (int i = 0; i < softStartIterations; i++) {
            xVel *= Math.pow(100, 1 / softStartIterations);
            yVel *= Math.pow(100, 1 / softStartIterations);
            angularVel *= Math.pow(100, 1 / softStartIterations);

            drive(xVel, yVel, angularVel, false);

            Utils.sleep(softStopSleepTime);
        }
    }

    public String drive(double xVel, double yVel, double angularVel, boolean isChannelMode) { //translate/rotate the robot given velocity and angular velocity, and return a string representation of this algorithm

        xVel = -xVel; //this is a fix for some trig bug (deleting this statement and changing 'Math.cos(theta - Math.PI / 4)' to 'Math.cos(theta + Math.PI / 4)' should do the trick

        if (isChannelMode) { //in channel mode negligible velocities will be disregarded
            xVel = Math.abs(xVel) < channelModeStickThreshold ? 0 : xVel;
            yVel = Math.abs(yVel) < channelModeStickThreshold ? 0 : yVel;
            angularVel = Math.abs(angularVel) < channelModeStickThreshold ? 0 : angularVel;
        }

        double robotVel = Utils.trim(0, 1, Utils.getMagnitude(xVel, yVel)), //magnitude of robot velocity [0, 1]
                theta = Utils.atan3(yVel, xVel); //robot translation angle [0, 2π]

        double[][] relativeWheelVels = Utils.multiplyValues(robotVel, ////multiply the scaled values by the desired robot velocity
                Utils.scaleValues(0, new double[][] {{ //scale velocities to unit vectors [-1, 1]
                    Math.sin(theta - Math.PI / 4), //front left and back right wheel relative velocity (without angular velocity)
                    Math.cos(theta - Math.PI / 4) //front right and back left wheel relative velocity (without angular velocity)
        }}));

        //return a string representation of the algorithm, and power the wheels after scaling and accounting for angular velocity
        return "vel: " + Utils.toString(robotVel) +
                ", theta: " + Utils.toString(theta) +
                ", ngVel: " + Utils.toString(angularVel) +
                ", mode: " + (isChannelMode ? "chan" : "prec") +
                ", pow: " +
                setMotorPowers( //apply the scaled powers to the motors
                        Utils.scaleValues(1, new double[][] { //scale the velocities to unit vectors, only if any of them have a magnitude greater than one
                                {
                                        compensationConstants[0][0] * (relativeWheelVels[0][0] + angularVel), //front left wheel relative velocity (with angular velocity)
                                        compensationConstants[0][1] * (relativeWheelVels[0][1] - angularVel) //front right wheel relative velocity (with angular velocity)
                                }, {
                                        compensationConstants[1][0] * (relativeWheelVels[0][1] + angularVel), //back left wheel relative velocity (with angular velocity)
                                        compensationConstants[1][1] * (relativeWheelVels[0][0] - angularVel) //back right wheel relative velocity (with angular velocity)
                                }
                        })
                );
    }

    private String setMotorPowers(double[][] wheelPowers) { //apply power to the motors, and return string representation of the wheel powers
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            wheelbase[i][j].setPower(wheelPowers[i][j]);

        return "{ [" + Utils.toString(wheelPowers[0][0]) + ", " + Utils.toString(wheelPowers[0][1]) + "], [" + Utils.toString(wheelPowers[1][0]) + ", " + Utils.toString(wheelPowers[1][1]) + "] }";
    }

    public void stop() { //stop the robot
        for (DcMotor[] a : wheelbase) for (DcMotor b : a)
            b.setPower(0);
    }

    public void softStop() {
        while (stillMovingSignificantly()) {
            setMotorPowers(Utils.multiplyValues(softStopPercentage, getMotorPowers()));
            Utils.sleep(softStopSleepTime);
        }
        stop();
    }

    private boolean stillMovingSignificantly() {
        return Utils.getMaxMagnitude(getMotorPowers()) > significantMovementThreshold;
    }

    private double[][] getMotorPowers() {
        double[][] powers = new double[2][2];

        for (int i = 0; i < wheelbase.length; i++)
            for (int j = 0; j < wheelbase[i].length; j++)
                powers[i][j] = wheelbase[i][j].getPower();

        return powers;
    }

}
