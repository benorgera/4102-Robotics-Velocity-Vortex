package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.opmodes.AutonomousImplementation;
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
    private ColorSensor[] lineSensors;
    private ModernRoboticsAnalogOpticalDistanceSensor ods;
    private ModernRoboticsI2cColorSensor[] beaconSensors;
    private VoltageSensor voltage;
    private ButtonPusher buttonPusher;
    private ModernRoboticsI2cRangeSensor rangeSensor;

    private Integrator integrator;

    private Wheels wheels;

    private double initialHeading = 0;

    private final long sensorLoopLatency = 0;

    private final double headingAccuracyThreshold = 1 * Math.PI / 180; //1 degrees

    private double strafeConstant = 1.71; //amount of extra power supplied based on horizontal component of translation angle (strafing takes more power)
    private double ngConstant = 0.6; //rotational speed given based on deviation from 0 degrees

    private Thread gyroPoll;

    public Sensors(BNO055IMU imu, ColorSensor[] lineSensors, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor[] beaconSensors, TouchSensor[] touchSensors, ModernRoboticsI2cRangeSensor rangeSensor, VoltageSensor voltage) {
        this.wheels = Hardware.getWheels();
        this.buttonPusher = Hardware.getButtonPusher();
        this.imu = imu;
        this.voltage = voltage;
        this.ods = ods;
        this.rangeSensor = rangeSensor;
        this.touchSensors = touchSensors;
        this.lineSensors = lineSensors;
        this.beaconSensors = beaconSensors;

        for (ModernRoboticsI2cColorSensor m : beaconSensors)
            m.enableLed(false);
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

    //left then right, the more positive the more red
    public int[] getBeaconReadings() {
        return new int[] {
                beaconSensors[0].red() - beaconSensors[0].blue(), //left
                beaconSensors[1].red() - beaconSensors[1].blue() //right
        };
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
        compensatedTranslate(thetaDesired, 0.25);
    }

    public void centerOnZero() {
        double heading = getHeading(),
                previousHeading = heading,
                power = 0.13;
        long checkTime = System.currentTimeMillis() + 1000;

        while (Hardware.active() && Math.abs(heading) > headingAccuracyThreshold) {
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

        initialHeading = (storedHeading - theta) % (2 * Math.PI);      //sets our heading to was it was initally plus how much we should have turned
    }

    public void parallelPark(double thetaTranslateFinal, double deltaThetaTranslate, double translateSpeed, double deltaTranslateTolerance, double turnTheta, double turnSpeed, double turnTolerance, long speedTimeout) { //positive is counterclockwise, max turn is PI

        boolean isCounterClockwise = turnTheta > 0;

        //add tolerance change in translation angle (measured from final), so translate is slightly towards wall (this causes the parallel park effect)
        deltaThetaTranslate += deltaTranslateTolerance * (deltaThetaTranslate > 0 ? -1 : 1);

        double storedHeading = initialHeading,
                threshold = turnTheta + turnTolerance * (isCounterClockwise ? -1 : 1),  //allows for the turn to stop a little before it turns fully, helps prevent overshooting angle
                thetaRemaining;

        long start = System.currentTimeMillis();

        resetHeading();

        speedTimeout += System.currentTimeMillis();

        //while we haven't reached our threshold yet, or its been too little time (<300 ms), turn towards the threshold
        while (Hardware.active() && ((thetaRemaining = Utils.difference(-getHeading(), threshold, !isCounterClockwise)) > 0 || System.currentTimeMillis() - start < 300)) {
            if (System.currentTimeMillis() > speedTimeout) {
                translateSpeed += 0.043;
                turnSpeed += 0.043;
                speedTimeout += 1500;
                Hardware.print("Speed timeout, upped translate/turn speed to " + translateSpeed);
            }
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
                lineSensors[0].green(), //green is a measure of white
                lineSensors[1].green()
        };
    }

    //returns false if beacon was already claimed, so if false is returned and this is the second beacon, we likely have another beacon to claim
    public boolean captureBeacon(boolean isRed, boolean isFirstRun, AutonomousImplementation a) {
        boolean[] colors = senseBeaconColors();

        if (colors[0] != colors[1] || isRed != colors[0]) //this is the check to be sure, and we have run twice and haven't claimed, lets hug wall
            a.hugWall();

        if (colors[0] != colors[1]) { //the buttons are different, the beacon is unclaimed
            Hardware.print("Claiming unclaimed beacon");
            buttonPusher.push(isRed == colors[0]); //push the proper button
            captureBeacon(isRed, false, a); //make sure the beacon has been claimed
        } else if (isRed != colors[0]) { //the buttons are the same color, but the beacon is claimed the wrong color
            Hardware.print("Reclaiming beacon, but must wait 5s first");
            Hardware.sleep(5000);
            Hardware.print("Reclaiming beacon");
            buttonPusher.pushBoth(); //push both, we just need to reverse the color
            captureBeacon(isRed, false, a); //make sure the beacon has been claimed
        } else {
            Hardware.print("Beacon already claimed"); //already claimed our color
        }

        return colors[0] != colors[1]; //returns whether the beacon was previously unclaimed
    }

    private boolean[] senseBeaconColors() { //left then right, red is true
        long stop = System.currentTimeMillis() + 600;

        int[] sums = new int[] {0, 0};

        while (Hardware.active() && System.currentTimeMillis() < stop) {
            int[] readings = getBeaconReadings();
            sums[0] += readings[0]; //left
            sums[1] += readings[1]; //right
            Hardware.sleep(sensorLoopLatency);
        }

        return new boolean[] {sums[0] > 0, sums[1] > 0};
    }

    //translates at a given theta and speed until we reach a white line
    public boolean driveUntilLineReadingThreshold(double theta, boolean shouldPollGyro, boolean shouldStop, long minTime, long maxTime, double speed, double whiteLineSignalThreshold, int readings) {
        long time = System.currentTimeMillis();
        int count = 0;

        maxTime += time;    //if greater than this, we are taking to long and should timeout
        minTime += time;    //if less than this, we haven't gone long enough (maybe we re-read a line we were already on)

        if (shouldPollGyro) readyCompensatedTranslate(0);

        //drive while we haven't gotten enough readings which meet our threshold or we haven't driven for long enough, and we haven't driven for too long
        while (Hardware.active() && ((time = System.currentTimeMillis()) < minTime || count < readings) && time < maxTime) {
            if (Utils.getMaxMagnitude(getLineReadings()) > whiteLineSignalThreshold && time > minTime) count++;
            compensatedTranslate(theta, speed);
            Hardware.sleep(sensorLoopLatency);
        }

        if (shouldStop) wheels.stop();

        if (shouldPollGyro) stopCompensatedTranslate();

        return time < maxTime;      //if we actually got to the line, or just timed out
    }

    //just drive by time, no sensor values needed
    public void driveByTime(double theta, long time, boolean shouldStop, double speed) {
        long stop = System.currentTimeMillis() + time;
        while (Hardware.active() && System.currentTimeMillis() < stop)
            compensatedTranslate(theta, speed);
        if (shouldStop) wheels.stop();
    }

    public boolean touchSensorPressed(boolean isIntakeSide) {
        return touchSensors[isIntakeSide ? 1 : 0].isPressed();
    }

    public int getRange() {
        return (int) rangeSensor.getDistance(DistanceUnit.CM);
    }

    public void driveUntilLineOrTouchOrRange(double velFar, double velClose, boolean isRed, long minTime, long speedTimeout, long maxTime, double whiteLineSignalThreshold, int rangeThresholdFar, int rangeThresholdTouching, int readings) {

        boolean isClose = false;

        maxTime += System.currentTimeMillis();
        minTime += System.currentTimeMillis();
        speedTimeout += System.currentTimeMillis();
        long time; //since I've seen your smile (-Chance the Rapper)

        int count = 0;

        while (Hardware.active() && !touchSensorPressed(isRed) && count < readings) {
            if ((time = System.currentTimeMillis()) > speedTimeout && time < maxTime) {
                velClose += 0.043;
                speedTimeout += 1500;
                Hardware.print("Speed timeout, upped velClose to " + velClose);
            }

            if (time > minTime && getRange() < rangeThresholdTouching)
                count++;

            compensatedTranslate(Math.PI / 2 * (isRed ? 1 : -1), ((isClose = time > minTime && (isClose || Utils.getMaxMagnitude(getLineReadings()) > whiteLineSignalThreshold) || getRange() < rangeThresholdFar)) ? velClose : velFar);
            Hardware.sleep(sensorLoopLatency);
        }

        wheels.stop();
    }

    public void driveUntilOdsThreshold(double theta, double threshold, int readings, double speed, long timeout) {
        timeout += System.currentTimeMillis();

        int count = 0;

        while (Hardware.active() && System.currentTimeMillis() < timeout && count < readings) {
            if (getOpticalDistance() > threshold) count++;
            compensatedTranslate(theta, speed);
            Hardware.sleep(sensorLoopLatency);
        }
        wheels.stop();
    }
}
