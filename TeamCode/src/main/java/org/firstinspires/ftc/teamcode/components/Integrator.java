package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.opengl.shaders.PositionAttributeShader;

/**
 * Created by benorgera on 11/25/16.
 */

public class Integrator implements BNO055IMU.AccelerationIntegrator {

    private Acceleration acceleration;
    private Velocity velocity;
    private Position position;

    private Acceleration previousAcceleration;
    private Velocity previousVelocity;

    private BNO055IMU.Parameters params;

    @Override
    public void initialize(BNO055IMU.Parameters params, Position initialPosition, Velocity initialVelocity) {
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.params = params;
    }

    @Override
    public Position getPosition() {
        return position;

    }

    @Override
    public Velocity getVelocity() {
        return velocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        acceleration = linearAcceleration;

        if (acceleration.acquisitionTime == 0) return; //zero means we don't know when this was acquired

        double deltaT = acceleration.acquisitionTime - previousAcceleration.acquisitionTime; //the width of the riemann sum

        velocity.xVeloc += Utils.average(acceleration.xAccel, previousAcceleration.xAccel) * deltaT;
        velocity.yVeloc += Utils.average(acceleration.yAccel, previousAcceleration.yAccel) * deltaT;
        velocity.zVeloc += Utils.average(acceleration.zAccel, previousAcceleration.zAccel) * deltaT;

        position.x += Utils.average(velocity.xVeloc, previousVelocity.xVeloc) * deltaT;
        position.y += Utils.average(velocity.yVeloc, previousVelocity.yVeloc) * deltaT;
        position.z += Utils.average(velocity.zVeloc, previousVelocity.zVeloc) * deltaT;

        previousAcceleration = acceleration;
        previousVelocity = velocity;

    }
}
