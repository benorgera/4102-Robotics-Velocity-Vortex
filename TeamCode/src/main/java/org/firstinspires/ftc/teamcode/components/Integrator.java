package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * {@link Integrator} is an integrator that doesn't actually
 * integrate accelerations, but merely reports them in the logcat log. This is a debugging
 * and demonstration tool, little more.
 */
public class Integrator implements BNO055IMU.AccelerationIntegrator
{
    private Acceleration acceleration;
    private Velocity velocity;
    private Position position;

    private Acceleration previousAcceleration;
    private Velocity previousVelocity;

    private BNO055IMU.Parameters parameters;

    @Override public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity)
    {
        this.parameters = parameters;
        this.position = initialPosition;
        this.velocity = initialVelocity;

        previousVelocity = new Velocity();
        previousAcceleration = new Acceleration();
        acceleration = new Acceleration();
    }

    @Override public Position getPosition() { return position == null ? new Position() : position.toUnit(DistanceUnit.METER); }
    @Override public Velocity getVelocity() { return velocity == null ? new Velocity() : velocity.toUnit(DistanceUnit.METER); }
    @Override public Acceleration getAcceleration()
    {
        return this.acceleration == null ? new Acceleration() : this.acceleration.toUnit(DistanceUnit.METER);
    }

    @Override public void update(Acceleration linearAcceleration)
    {
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

    public void reset() {
        velocity = new Velocity();
        position = new Position();
    }
}
