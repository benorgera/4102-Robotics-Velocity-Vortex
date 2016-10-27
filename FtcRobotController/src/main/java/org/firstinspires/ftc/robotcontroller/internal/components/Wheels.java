package org.firstinspires.ftc.robotcontroller.internal.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by benorgera on 10/24/16.
 */
public class Wheels {
    private DcMotor[][] wheelBase;

    //constants
    private final double stickThreshold = 0.1;
    private final double[][] compensationConstants = {
            {1, 1},
            {1, 1}
    };

    public Wheels(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {

        this.wheelBase = new DcMotor[][] { //initialize motors
                {frontLeft, frontRight},
                {backLeft, backRight}
        };

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) { //set up motors
            wheelBase[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheelBase[i][j].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public String drive(double xVel, double yVel, double angularVel, boolean isChannelMode) { //translate the robot given velocity and angular velocity, and return a string representation of this algorithm

        if (isChannelMode) { //in channel mode negligible velocities will be disregarded
            xVel = Math.abs(xVel) < stickThreshold ? 0 : xVel;
            yVel = Math.abs(yVel) < stickThreshold ? 0 : yVel;
            angularVel = Math.abs(angularVel) < stickThreshold ? 0 : angularVel;
        }

        double robotVel = Utils.trim(0, 1, Utils.getMagnitude(xVel, yVel)), //magnitude of robot velocity [0, 1]
                theta = Utils.atan3(yVel, xVel); //robot translation angle [0, 2π]

        double[][] relativeWheelVelocities = Utils.multiplyValues(robotVel, ////multiply the scaled values by the desired robot velocity
                Utils.scaleValues(0, new double[][] {{ //scale velocities to unit vectors [-1, 1]
                    Math.sin(theta - Math.PI / 4), //front left and back right wheel relative velocity (without angular velocity)
                    Math.cos(theta - Math.PI / 4) //front right and back left wheel relative velocity (without angular velocity)
        }}));

        return "vel: " + Utils.toString(robotVel) +
            ", theta: " + Utils.toString(theta) +
            ", ngVel: " + Utils.toString(angularVel) +
            ", mode: " + (isChannelMode ? "chan" : "prec") +
            ", pow: " +
                setWheelPowers( //apply power to the motors
                    Utils.scaleValues(1, new double[][] { //scale the vectors to unit vectors, only if any of them are over one
                        {
                            compensationConstants[0][0] * (relativeWheelVelocities[0][0] + angularVel), //front left wheel relative velocity (with angular velocity)
                            compensationConstants[0][1] * (relativeWheelVelocities[0][1] - angularVel) //front right wheel relative velocity (with angular velocity)
                        }, {
                            compensationConstants[1][0] * (relativeWheelVelocities[0][1] + angularVel), //back left wheel relative velocity (with angular velocity)
                            compensationConstants[1][1] * (relativeWheelVelocities[0][0] - angularVel) //back right wheel relative velocity (with angular velocity)
                        }
                    })
                );
    }


    private String setWheelPowers(double[][] wheelPowers) { //apply power to the motors, and return string representation of the wheel powers
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
                wheelBase[i][j].setPower(wheelPowers[i][j]);

        return "{ [" + Utils.toString(wheelPowers[0][0]) + ", " + Utils.toString(wheelPowers[0][1]) + "], [" + Utils.toString(wheelPowers[1][0]) + ", " + Utils.toString(wheelPowers[1][1]) + "] }";
    }

    public void stop() { //stop the robot
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            wheelBase[i][j].setPower(0);
    }

}
