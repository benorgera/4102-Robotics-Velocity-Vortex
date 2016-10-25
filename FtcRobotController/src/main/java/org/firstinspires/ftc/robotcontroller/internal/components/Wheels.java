package org.firstinspires.ftc.robotcontroller.internal.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.internal.Utils;

/**
 * Created by benorgera on 10/24/16.
 */
public class Wheels {
    private DcMotor[][] wheelBase;

    //constants
    private final double stickThreshold = 0.1;
    private final double stickRadius = 1;
    private final double[][] compensationConstants = new double[][] {
            {1, 1},
            {1, 1}
    };

    public Wheels(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {

        this.wheelBase = new DcMotor[][] { //initialize motors
                {frontLeft, frontRight},
                {backLeft, backRight}
        };

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) { //set up motors
            wheelBase[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheelBase[i][j].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public String drive(double xVel, double yVel, double angularVel, boolean isChannelMode) { //drive robot given x velocity vector, y velocity vector and angular velocity vector, disregarding negligible

        //trim sticks to unit vectors
        xVel /= stickRadius;
        yVel /= stickRadius;
        angularVel /= stickRadius;

        if (isChannelMode) { //in channel mode negligible velocities will be disregarded
            xVel = Math.abs(xVel) < stickThreshold ? 0 : xVel;
            yVel = Math.abs(yVel) < stickThreshold ? 0 : yVel;
            angularVel = Math.abs(angularVel) < stickThreshold ? 0 : angularVel;
        }

        double vel = Utils.getMagnitude(xVel, yVel), //unit vector
                theta = Utils.atan3(yVel, xVel); //angle [0, 2π]

        drive2(vel, theta, angularVel);

        return "vel: " + Utils.toString(vel) + ", theta: " + Utils.toString(theta) + ", angularVel: " + Utils.toString(angularVel) + ", mode: " + (isChannelMode ? "channel" : "precise");
    }

    private void drive2(double robotVelocity, double robotAngle, double angularVelocity) { //translate robot at given velocity and angle with given angular velocity
        setWheelPowers(scaleWheelPowers(new double[][]{
            {
                compensationConstants[0][0] * (robotVelocity * Math.sin(robotAngle + Math.PI / 4) + angularVelocity), //front left wheel
                compensationConstants[0][1] * (robotVelocity * Math.cos(robotAngle + Math.PI / 4) - angularVelocity) //front right wheel
            }, {
                compensationConstants[1][0] * (robotVelocity * Math.cos(robotAngle + Math.PI / 4) + angularVelocity), //back left wheel
                compensationConstants[1][1] * (robotVelocity * Math.sin(robotAngle + Math.PI / 4) - angularVelocity) //back right wheel
            }
        }));
    }

    private void setWheelPowers(double[][] wheelPowers) { //apply power to the motors
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                wheelBase[i][j].setPower(wheelPowers[i][j]);
    }

    public void stop() { //stop the robot
        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            wheelBase[i][j].setPower(0);
    }

    private double[][] scaleWheelPowers(double[][] wheelPowers) { //scale wheel powers so robot can use maximum powers with any combination of rotation and translation
        double max = 0;

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) //find max wheel power
            if (Math.abs(wheelPowers[i][j]) > max)
                max = Math.abs(wheelPowers[i][j]);

        if (max <= 1) return wheelPowers; //return if no scaling is necessary

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) //scale the powers by the max value
            wheelPowers[i][j] /= max;

        return wheelPowers;
    }

}
