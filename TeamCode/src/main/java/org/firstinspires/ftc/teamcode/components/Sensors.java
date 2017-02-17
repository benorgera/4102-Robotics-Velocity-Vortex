package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.utilities.GyroPoll;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 10/25/16.
 */
public class Sensors {

    private Orientation angles;

    private BNO055IMU imu;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private ModernRoboticsAnalogOpticalDistanceSensor ods;
    private ModernRoboticsI2cColorSensor beaconSensor;
    private VoltageSensor voltage;

    private Integrator integrator;

    private Wheels wheels;

    private double initialHeading = 0;

    private final double compensatedTranslateSpeed = 0.25;

    private final double headingAccuracyThreshold = 1 * Math.PI / 180; //1 degrees

    private double strafeConstant = 1.71; //amount of extra power supplied based on horizontal component of translation angle (strafing takes more power)
    private double ngConstant = 0.6; //rotational speed given based on deviation from 0 degrees

    private Thread gyroPoll;

    public Sensors(BNO055IMU imu, ColorSensor leftColorSensor, ColorSensor rightColorSensor, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor, VoltageSensor voltage) {
        this.imu = imu;
        this.voltage = voltage;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.ods = ods;
        this.wheels = Hardware.getWheels();
        this.beaconSensor = beaconSensor;
        beaconSensor.enableLed(false);
    }

    public void initImu() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.NDOF;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.temperatureUnit     = BNO055IMU.TempUnit.CELSIUS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = integrator = new Integrator();

        imu.initialize(parameters);

        //remap the axes to the orientation the imu is mounted in
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, 0x21);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, 0x01);

        resetHeading();
    }

    public double getHeading() { // [-π, π]
        return Utils.angleDifference(getRawHeading(), initialHeading);
    }

    public void resetHeading() {
        initialHeading = getRawHeading();
    }

    public double getRawHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return -AngleUnit.RADIANS.normalize(AngleUnit.RADIANS.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public Velocity getVelocity() {
        return imu.getVelocity();
    }

    public Position getPosition() {
        return imu.getPosition();
    }

    public double getOpticalDistance() {
        return ods.getLightDetected();
    }

    public int[] getBeaconColor() {
        return new int[] {beaconSensor.blue(), beaconSensor.red()};
    }

    public void resetIntegrator() {
        integrator.reset();
    }

    public double getInitialHeading() {
        return initialHeading;
    }

    public void compensatedTranslate(double thetaDesired, double speed) { //translate robot with rotation compensation (must be called on a loop)
        double power = (speed + getVoltageConstant(speed)) * (1 + strafeConstant * Math.abs(Math.cos(thetaDesired))),
                ngVel = Utils.trim(-1, 1, -1 * getHeading() * ngConstant); //compensate for rotation by accounting for change in heading



        wheels.drive(Math.cos(thetaDesired) * power, Math.sin(thetaDesired) * power, ngVel, false);
    }

    public void compensatedTranslate(double thetaDesired) { //translate robot with rotation compensation (must be called on a loop)
        compensatedTranslate(thetaDesired, compensatedTranslateSpeed);
    }

    public void centerOnZero() {
        double heading = getHeading(),
                previousHeading = heading,
                power = 0.13;
        long checkTime = System.currentTimeMillis() + 1000;

        while (Math.abs(heading) > headingAccuracyThreshold && Hardware.active()) {
            wheels.drive(0, 0, power * ((heading = getHeading()) > 0 ? -1 : 1), false);

            if (System.currentTimeMillis() >= checkTime) { //if its been 250 ms, consider upping the power and ready the next check

                if (Math.abs(previousHeading - heading) < headingAccuracyThreshold) //if the robot hasn't rotated at 1 deg / s
                    power += 0.03;

                previousHeading = heading;
                checkTime = System.currentTimeMillis() + 1000;
            }
        }
        wheels.stop();
    }

    private double getVoltageConstant(double speed) {
        double minVoltage = 12.8,
                maxVoltage = 13.7,
                range = maxVoltage - minVoltage,
                maxAdditionalSpeed = 0.06,
                v = voltage.getVoltage(),
                difVolt = (maxVoltage - v) / range,
                difSpeed = Utils.trim(0, 1, speed / compensatedTranslateSpeed);

        return v >= maxVoltage ? 0 : (difVolt * maxAdditionalSpeed * difSpeed);
    }

    public void turn(double theta, double speed) { //positive is clockwise, max turn is PI

//        boolean isCounterClockwise = theta < 0;
//        theta = Math.abs(theta);
//        boolean isTurningAround = theta == Math.PI,
//                isGreaterThan = isTurningAround ? !isCounterClockwise : isCounterClockwise;
//
//        double storedHeading = initialHeading,
//                threshold = isTurningAround ? 0 : theta;
//        long start = System.currentTimeMillis();
//
//        resetHeading();
////
////        while (Hardware.active() && Utils.compare(getHeading(), threshold, isGreaterThan)) {
////
////        }
//
//        while (Hardware.active() && (theta == Math.PI ? (isCounterClockwise ? (getHeading()) < 0 : (getHeading() > 0)) : (Math.abs(getHeading()) < theta)) || System.currentTimeMillis() - start < 300)
//            wheels.drive(0, 0, (isCounterClockwise ? -1 : 1) * speed * (theta == Math.PI ? (Math.PI - Math.abs(getHeading()) < thresh ? ) : theta - Math.abs(getHeading())), false);
//
//        wheels.stop();
//
//        initialHeading = (storedHeading + getHeading()) % (2 * Math.PI);

        boolean isCounterClockwise = theta < 0;
        theta = Math.abs(theta);

        double storedHeading = initialHeading;
        long start = System.currentTimeMillis();

        resetHeading();

        while (Hardware.active() && (theta == Math.PI ? (isCounterClockwise ? (getHeading() < 0) : (getHeading() > 0)) : (Math.abs(getHeading()) < theta)) || System.currentTimeMillis() - start < 300)
            wheels.drive(0, 0, (isCounterClockwise ? -1 : 1) * speed, false);

        wheels.stop();

        initialHeading = (storedHeading + getHeading()) % (2 * Math.PI);
    }

    public double getStrafeConstant() {
        return strafeConstant;
    }

    public void setStrafeConstant(double s) {
        strafeConstant = s;
    }

    public double getNgConstant() {
        return ngConstant;
    }

    public void setNgConstant(double n) {
        ngConstant = n;
    }

    private void readyCompensatedTranslate(long softStartTime) {
        wheels.readyCompensatedTranslate(softStartTime);
        (gyroPoll = new Thread(new GyroPoll(imu))).start();
    }

    private void stopCompensatedTranslate() {
        gyroPoll.interrupt();
        wheels.stopCompensatedTranslating();
    }

    public double[] getLineReadings() {
        return new double[] {
                leftColorSensor.green(), //green is a measure of white
                rightColorSensor.green()
        };
    }

    private void followLine(double speed, boolean isGoingForwards) {
        double[] readings = getLineReadings();
        double left = readings[0],
                right = readings[1];

        if (Math.abs(left - right) <= 50) { //both sensors equally on the white line
            compensatedTranslate(isGoingForwards ? Math.PI : 0, speed);
        } else { //left more on the white line turn left, right turn right
            compensatedTranslate((isGoingForwards ? Math.PI : 0) + (Math.PI / 9 * (left > right ? 1 : -1) * (isGoingForwards ? 1 : -1)), speed);
        }
    }

    public void findBeaconButton(boolean isRed, double whiteLineThreshold, double speed) {

        boolean goingForward = true,
                shouldRestart = false;

        double previousReading = getBeaconColor()[isRed ? 1 : 0],
                currentReading,
                maxColor = 0,
                directionSwitches = 0;

        long lastDirectionSwitch = System.currentTimeMillis();

        while (Hardware.active()) {

            if ((currentReading = getBeaconColor()[isRed ? 1 : 0]) > maxColor)
                maxColor = currentReading;

            if (directionSwitches >= 3 && currentReading >= maxColor) {
                wheels.stop();
                return;
            } else if (directionSwitches > 5) {
                maxColor--;
                directionSwitches = 4;
            }

            if (System.currentTimeMillis() - lastDirectionSwitch > 3000) {
                Hardware.print("Restarted find beacon button");
                driveUntilLineReadingThreshold(Math.PI / 2 * (goingForward ? 3 : 1), whiteLineThreshold, false, speed);
                shouldRestart = true;
                break;
            } else if (currentReading < previousReading && System.currentTimeMillis() - lastDirectionSwitch > 300) {
                goingForward = !goingForward;
                directionSwitches++;
                lastDirectionSwitch = System.currentTimeMillis();
            }

            Hardware.clearLog();
            Hardware.print("Reading: " + currentReading);
            Hardware.print("Direction switches: " + directionSwitches);
            Hardware.print("Threshold: " + maxColor);

            compensatedTranslate(Math.PI / 2 * (goingForward ? 1 : 3), speed);

            previousReading = currentReading;
        }

        if (shouldRestart) findBeaconButton(isRed, whiteLineThreshold, speed);
    }

    public void findBeaconButton(boolean isRed, double whiteLineReadingThreshold) {
        findBeaconButton(false, whiteLineReadingThreshold, 0.125);
    }

    public void driveUntilOdsThreshold(double odsThreshold, double speed) {
        boolean isGoingForwards = getOpticalDistance() < odsThreshold;

        while (Hardware.active() && isGoingForwards ? (getOpticalDistance() < odsThreshold) : (getOpticalDistance() > odsThreshold))
            compensatedTranslate(isGoingForwards ? Math.PI : 0, speed);
        wheels.stop();
    }

    public void driveUntilOdsThreshold(double odsThreshold) {
        driveUntilOdsThreshold(odsThreshold, compensatedTranslateSpeed);
    }

    public void followLineUntilOdsThreshold(double odsThreshold, double speed) {
        boolean isGoingForwards = getOpticalDistance() < odsThreshold;

        while (Hardware.active() && Utils.compare(getOpticalDistance(), odsThreshold, !isGoingForwards))
            followLine(speed, isGoingForwards);
        wheels.stop();
    }

    public void followLineUntilOdsThreshold(double odsThreshold) {
        followLineUntilOdsThreshold(odsThreshold, compensatedTranslateSpeed);
    }


    public void driveUntilLineReadingThreshold(double theta, double whiteLineReadingThreshold, boolean shouldPollGyro, double speed) {
        if (shouldPollGyro) readyCompensatedTranslate(0);

        int sufficientReadings = 0;

        while (Hardware.active() && sufficientReadings < 2) {
            compensatedTranslate(theta, speed);
            if (Utils.getMaxMagnitude(getLineReadings()) > whiteLineReadingThreshold)
                sufficientReadings++;
        }
        wheels.stop();

        if (shouldPollGyro) stopCompensatedTranslate();
    }

    public void driveUntilLineReadingThreshold(double theta, double whiteLineReadingThreshold, boolean shouldPollGyro) {
        driveUntilLineReadingThreshold(theta, whiteLineReadingThreshold, shouldPollGyro, compensatedTranslateSpeed);
    }

    public void driveByTime(double theta, long time, boolean shouldStop, double speed) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            compensatedTranslate(theta, speed);
        if (shouldStop) wheels.stop();
    }

    public void driveByTime(double theta, long time, boolean shouldStop) {
        driveByTime(theta, time, shouldStop, compensatedTranslateSpeed);
    }

    public void uncompensatedDriveByTime(double xVel, double yVel, double ngVel, long time) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            wheels.drive(xVel, yVel, ngVel, false);
        wheels.stop();
    }

    public void turnByTime(long time, double ngVel) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            wheels.drive(0, 0, ngVel, false);
        wheels.stop();
    }
}
