package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 10/24/16.
 */
public class Wheels {
    private DcMotor[][] wheelbase;

    private boolean isCompensatedTranslating = false;

    //constants
    private final double channelModeStickThreshold = 0.1; //the max magnitude of a stick reading

    private long softStartLength; //ms to bring robot to full speed
    private long softStartStartTime; //ms time when movement started
    private boolean isSoftStarting = false;


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
        wheelbase[0][1].setDirection(DcMotor.Direction.REVERSE);
        wheelBase[1][1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void readyCompensatedTranslate(long softStartTime) {
        if (softStartTime > 0) {
            softStartLength = softStartTime;
            softStartStartTime = System.currentTimeMillis();
            isSoftStarting = true;
        }

        isCompensatedTranslating = true;
    }

    public void stopCompensatedTranslating() {
        isCompensatedTranslating = false;
    }

    private double getSoftStartScalar() {
        double scalar = ((double) System.currentTimeMillis() - (double) softStartStartTime) / (double) softStartLength;

        if (scalar >= 1) isSoftStarting = false;

        return Utils.trim(0, 1, scalar);
    }

    public void drive(double xVel, double yVel, double angularVel, boolean isChannelMode) { //translate/rotate the robot given velocity and angular velocity, and return a string representation of this algorithm

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

        //power the wheels after scaling and accounting for angular velocity
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
        if (isCompensatedTranslating)
            wheelPowers = Utils.multiplyValues(Hardware.getGyroConstant(), wheelPowers);
        if (isSoftStarting)
            wheelPowers = Utils.multiplyValues(getSoftStartScalar(), wheelPowers);

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            wheelbase[i][j].setPower(wheelPowers[i][j]);

        return "{ [" + Utils.toString(wheelPowers[0][0]) + ", " + Utils.toString(wheelPowers[0][1]) + "], [" + Utils.toString(wheelPowers[1][0]) + ", " + Utils.toString(wheelPowers[1][1]) + "] }";
    }

    public void stop() { //stop the robot
        for (DcMotor[] a : wheelbase) for (DcMotor b : a)
            b.setPower(0);
    }

    public void softStop(long time) {   //gradually slows down a drive over time

        long stop = System.currentTimeMillis() + time;
        double[][] motorPowers = getMotorPowers();

        while (System.currentTimeMillis() < stop)
            setMotorPowers(Utils.multiplyValues((stop - System.currentTimeMillis()) / time, motorPowers));

        stop();
    }

    private double[][] getMotorPowers() {
        double[][] powers = new double[2][2];

        for (int i = 0; i < wheelbase.length; i++)
            for (int j = 0; j < wheelbase[i].length; j++)
                powers[i][j] = wheelbase[i][j].getPower();

        return powers;
    }

}
