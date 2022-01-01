package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    private TouchSensor[] touchSensors;
    private BNO055IMU imu;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private ModernRoboticsAnalogOpticalDistanceSensor ods;
    private ModernRoboticsI2cColorSensor beaconSensor;
    private VoltageSensor voltage;
    private ButtonPusher buttonPusher;

    private Integrator integrator;

    private Wheels wheels;

    private double initialHeading = 0;

    private final double compensatedTranslateSpeed = 0.25;

    private final double headingAccuracyThreshold = 1 * Math.PI / 180; //1 degrees

    private double strafeConstant = 1.71; //amount of extra power supplied based on horizontal component of translation angle (strafing takes more power)
    private double ngConstant = 0.6; //rotational speed given based on deviation from 0 degrees

    private Thread gyroPoll;

    public Sensors(BNO055IMU imu, ColorSensor leftColorSensor, ColorSensor rightColorSensor, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor, TouchSensor[] touchSensors, VoltageSensor voltage) {
        this.imu = imu;
        this.voltage = voltage;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.ods = ods;
        this.wheels = Hardware.getWheels();
        this.buttonPusher = Hardware.getButtonPusher();
        this.beaconSensor = beaconSensor;

        this.touchSensors = touchSensors;
        beaconSensor.enableLed(true);
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

    private double getVoltageConstant(double speed) {       //scales up motor powers based on how low our battery voltage is to make a more consistent drive
        double minVoltage = 12.8,
                maxVoltage = 13.7,
                range = maxVoltage - minVoltage,
                maxAdditionalSpeed = 0.05,
                v = voltage.getVoltage(),
                difVolt = (maxVoltage - v) / range,
                difSpeed = Utils.trim(0, 1, speed / 0.35);

        return v >= maxVoltage ? 0 : (difVolt * maxAdditionalSpeed * difSpeed);
    }

    public void turn(double theta, double tolerance, double speed) { //positive is counterclockwise, max turn is PI

        boolean isCounterClockwise = theta > 0;

        double storedHeading = initialHeading,
                threshold = theta + tolerance * (isCounterClockwise ? -1 : 1);      //allows for the turn to stop a little before it turns fully, helps prevent overshooting angle

        long start = System.currentTimeMillis();

        resetHeading();


        //while we haven't reached our threshold yet, or its been too little time (<300 ms), turn towards the threshold
        while (Hardware.active() && (Utils.compare(-getHeading(), threshold, !isCounterClockwise) || System.currentTimeMillis() - start < 300))
            wheels.drive(0, 0, (isCounterClockwise ? -1 : 1) * speed, false);

        wheels.stop();

        initialHeading = (storedHeading + -theta) % (2 * Math.PI);      //sets our heading to was it was initally plus how much we should have turned
    }

    public void parallelPark(double thetaTranslateFinal, double deltaThetaTranslate, double translateSpeed, double deltaTranslateTolerance, double turnTheta, double turnSpeed, double turnTolerance) { //positive is counterclockwise, max turn is PI

        boolean isCounterClockwise = turnTheta > 0;

        //add tolerance change in translation angle (measured from final), so translate is slightly towards wall (this causes the parallel park effect)
        deltaThetaTranslate += deltaTranslateTolerance * (deltaThetaTranslate > 0 ? -1 : 1);

        double storedHeading = initialHeading,
                threshold = turnTheta + turnTolerance * (isCounterClockwise ? -1 : 1),  //allows for the turn to stop a little before it turns fully, helps prevent overshooting angle
                thetaRemaining;

        long start = System.currentTimeMillis();

        resetHeading();

        //while we haven't reached our threshold yet, or its been too little time (<300 ms), turn towards the threshold
        while (Hardware.active() && ((thetaRemaining = Utils.difference(-getHeading(), threshold, !isCounterClockwise)) > 0 || System.currentTimeMillis() - start < 300)) {
            double proportionRemaining = Math.abs(thetaRemaining / threshold),
                    translateTheta = thetaTranslateFinal - deltaThetaTranslate * proportionRemaining;

            wheels.drive(translateSpeed * Math.cos(translateTheta), translateSpeed *  Math.sin(translateTheta), (isCounterClockwise ? -1 : 1) * turnSpeed, false);
        }

        wheels.stop();

        initialHeading = (storedHeading + -turnTheta) % (2 * Math.PI);      //sets our heading to was it was initally plus how much we should have turned
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

        if (Math.abs(left - right) <= 40) { //both sensors equally on the white line
            compensatedTranslate(isGoingForwards ? Math.PI + Math.PI / 30 : 0, speed);
        } else { //left more on the white line turn left, right turn right
            compensatedTranslate((isGoingForwards ? Math.PI : 0) + (Math.PI / 8 * (left > right ? 1.2 : -1) * (isGoingForwards ? 1 : -1)), speed);
        }
    }

    public void captureBeacon(boolean isRed) {
        buttonPusher.push(isRed == leftIsRed());
    }

    private boolean leftIsRed() {
        long stop = System.currentTimeMillis() + 500;

        double redTotal = 0,
                 blueTotal = 0;

        while (System.currentTimeMillis() < stop) {
            blueTotal += beaconSensor.blue();
            redTotal += beaconSensor.red();
            Hardware.sleep(10);
        }

        return redTotal > blueTotal;
    }


    public void driveUntilOdsThreshold(double odsThreshold, double speed) {     //drives until a specified ods value
        double reading = getOpticalDistance();
        boolean isGoingForwards = reading < odsThreshold;    //figure out if we need to go forwards or backwards

        //while we have not reached the ods threshold, and the difference between the 2 ods's is not large (meaning we lost the beacon)
        while (Hardware.active() && isGoingForwards ? ((reading = getOpticalDistance()) < odsThreshold) : (reading = getOpticalDistance()) > odsThreshold)
            compensatedTranslate(isGoingForwards ? Math.PI : 0, speed);
        wheels.stop();
    }

    public void pulseUntilOdsThreshold(double odsThreshold, double speed) {      //pulses until a specified ods value. tends to be straighter than drive
        double reading = getOpticalDistance();
        boolean isGoingForwards = reading < odsThreshold;    //figure out if we need to go forwards or backwards

        long start = System.currentTimeMillis();

        //while we have not reached the ods threshold, and the difference between the 2 ods's is not large (meaning we lost the beacon)
        while (Hardware.active() && isGoingForwards ? ((reading = getOpticalDistance()) < odsThreshold) : ((reading = getOpticalDistance()) > odsThreshold)) {
            if ((System.currentTimeMillis() - start) % 200 < 100)   //drive for 100 ms, stop for 100 ms
                compensatedTranslate(isGoingForwards ? Math.PI : 0, speed);
            else
                wheels.stop();
        }
        wheels.stop();
    }

    public void pulseUntilOdsThreshold(double odsThreshold) {
        pulseUntilOdsThreshold(odsThreshold, compensatedTranslateSpeed);
    }

    public void driveUntilOdsThreshold(double odsThreshold) {
        driveUntilOdsThreshold(odsThreshold, compensatedTranslateSpeed);
    }

    public void followLineUntilOdsThreshold(double odsThreshold, long timeout, boolean canGoBackwards, double speed) {  //follows the line until a specified ods value
        timeout += System.currentTimeMillis();
        double reading = getOpticalDistance();
        boolean isGoingForwards = reading < odsThreshold;

        if (!canGoBackwards && !isGoingForwards) return;

        while (Hardware.active() && Utils.compare((reading = getOpticalDistance()), odsThreshold, !isGoingForwards) && System.currentTimeMillis() < timeout)
            followLine(speed, isGoingForwards);
        wheels.stop();
    }

    public void followLineUntilOdsThreshold(double odsThreshold, long timeout, boolean canGoBackwards) {
        followLineUntilOdsThreshold(odsThreshold, timeout, canGoBackwards, compensatedTranslateSpeed);
    }


    //translates at a given theta and speed until we reach a white line
    public boolean driveUntilLineReadingThreshold(double theta, double whiteLineReadingThreshold, boolean shouldPollGyro, boolean shouldStop, long minTime, long maxTime, double speed) {
        long time = System.currentTimeMillis();

        maxTime += time;    //if greater than this, we are taking to long and should timeout
        minTime += time;    //if less than this, we haven't gone long enough (maybe we re-read a line we were already on)

        if (shouldPollGyro) readyCompensatedTranslate(0);

        int sufficientReadings = 0,     //number of readings that meet our threshold
                neededSufficientReadings = 1;   //number of readings that we need to consider ourdelves on the line, not a fluke

        //drive while we haven't gotten enough readings that meet our threshold, and the time we have been driving is between our min and max time
        while (Hardware.active() && (sufficientReadings < neededSufficientReadings && (time = System.currentTimeMillis()) < maxTime || (time = System.currentTimeMillis()) < minTime)) {
            compensatedTranslate(theta, speed);
            if (Utils.getMaxMagnitude(getLineReadings()) > whiteLineReadingThreshold)       //if we see the line, add 1 to sufficientReadings
                sufficientReadings++;
        }

        if (shouldStop) wheels.stop();

        if (shouldPollGyro) stopCompensatedTranslate();

        return time < maxTime;      //if we actually got to the line, or just timed out
    }

    public boolean driveUntilLineReadingThreshold(double theta, double whiteLineReadingThreshold, boolean shouldPollGyro, boolean shouldStop, long minTime, long maxTime) {
        return driveUntilLineReadingThreshold(theta, whiteLineReadingThreshold, shouldPollGyro, shouldStop, minTime, maxTime, compensatedTranslateSpeed);
    }

    //just drive by time, no sensor values needed
    public void driveByTime(double theta, long time, boolean shouldStop, double speed) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            compensatedTranslate(theta, speed);
        if (shouldStop) wheels.stop();
    }

    public void driveByTime(double theta, long time, boolean shouldStop) {
        driveByTime(theta, time, shouldStop, compensatedTranslateSpeed);
    }

    //just drive by time, no sensor values needed, and no compensation
    public void uncompensatedDriveByTime(double xVel, double yVel, double ngVel, long time) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            wheels.drive(xVel, yVel, ngVel, false);
        wheels.stop();
    }

    //turn for a time
    public void turnByTime(long time, double ngVel) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            wheels.drive(0, 0, ngVel, false);
        wheels.stop();
    }

    public void driveUntilTouchReading(double vel, boolean isRed) {
        while (Hardware.active() && !touchSensorPressed())
            compensatedTranslate(Math.PI / 2 * (isRed ? 1 : -1), vel);
    }

    public void driveUntilTouchReading(boolean isRed) {
        driveUntilTouchReading(0.6, isRed);
    }

    public boolean touchSensorPressed() {
        return touchSensors[0].isPressed() || touchSensors[1].isPressed();
    }

}
