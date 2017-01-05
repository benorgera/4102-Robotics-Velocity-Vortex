package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.Locale;

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

    private Integrator integrator;

    private Wheels wheels;

    private double initialHeading = 0;

    private final double translateSpeed = 0.2;

    private final double headingAccuracyThreshold = 1 * Math.PI / 180; //1.5 degrees

    private final double cyclesForMeaningfulData = 1000; //the number of cycles for compensation constant tweaks to be considered
    private final double ngConstantStep = 0.01; //the amount the constant is changed by each time a threshold is broken
    private double ngSignChangesPerCycleUpThreshold = 0.001; //the low number of sign changes per cycle needed to cause an ngConstant increase
    private double ngSignChangesPerCycleDownThreshold = 0.005; //the high number of sign changes per cycle needed to cause an ngConstant decrease

    private double strafeConstant = 1.71;
    private double ngConstant = 0.6;

    private double ngSignChanges = 0;
    private int translationCycles = 0;
    private boolean previousNgSignWasPositive = true;

    //color sensor representations
    private final String frontLeft = "[0, 0]";
    private final String frontRight = "[0, 1]";
    private final String backLeft = "[1, 0]";
    private final String backRight = "[1, 1]";

    public Sensors(BNO055IMU imu, ColorSensor leftColorSensor, ColorSensor rightColorSensor, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor) {
        this.imu = imu;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.ods = ods;
        this.wheels = Hardware.getWheels();
//        this.beaconSensor = beaconSensor;
//        beaconSensor.enableLed(false);
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
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, 0x24);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, 0x03);

        resetHeading();

//        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
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

    public void setInitialHeading(double heading) {
        initialHeading = heading;
    }

    public void resetCompensatedTranslate() {
        ngSignChanges = translationCycles = 0;
    }

    public String compensatedTranslate(double thetaDesired, boolean isFirstRun, boolean isExtraSlow) { //translate robot with rotation compensation (must be called on a loop)
        double power = translateSpeed * (1 + strafeConstant * Math.abs(Math.cos(thetaDesired))) * (isExtraSlow ? 0.48 : 1),
                ngVel = Utils.trim(-1, 1, -1 * getHeading() * ngConstant); //compensate for rotation by accounting for change in heading
        
//            thetaActual = Utils.atan3(sensors.getPosition().y, sensors.getPosition().x), //the direction of displacement thus far
//                thetaDif = thetaDesired - thetaActual, //the difference between desired and actual displacement direction
//                thetaNew = thetaDesired + thetaDif * k; //compensate for drift by accounting for the difference in desired and actual direction


//        String s = ngVel + " | " + ++translationCycles;
//
//        if (previousNgSignWasPositive != (previousNgSignWasPositive = ngVel > 0)) { //if there was a sign change in the ng (this if also stores the sign for next cycle)
//            ngSignChanges++;
//
//            double signChangesPerCycle = ngSignChanges / translationCycles;
//
//            if (signChangesPerCycle < ngSignChangesPerCycleUpThreshold && translationCycles > cyclesForMeaningfulData) { //we haven't changed sign enough times, let's up the compensation
//                ngConstant += ngConstantStep;
//                resetCompensatedTranslate();
//            } else if (signChangesPerCycle > ngSignChangesPerCycleDownThreshold && translationCycles > cyclesForMeaningfulData) { //we've changed sign too many times, let's down the compensation
//                ngConstant -= ngConstantStep;
//                resetCompensatedTranslate();
//            }
//        }

        if (isFirstRun) wheels.softStart(Math.cos(thetaDesired) * power, Math.sin(thetaDesired) * power,  0);

        wheels.drive(Math.cos(thetaDesired) * power, Math.sin(thetaDesired) * power, ngVel, false);

        return "" + ngVel;
    }

    public void centerOnZero() {
        double heading = getHeading();

        if (Math.abs(heading) > headingAccuracyThreshold) {
            double power = Utils.trim(0.05, 0.08, .04 * heading);

            wheels.drive(0, 0, power * (heading > 0 ? -1 : 1), false);
            Utils.sleep(15);
            wheels.stop();
            Utils.sleep(70);
        }
        wheels.stop();
    }

    public void turnAround() { //must call center on zero after
        double storedHeading = initialHeading;

        resetHeading();

        while (getHeading() > (-Math.PI / 2)) {
            wheels.drive(0, 0, -0.08, false);
        }

        wheels.stop();

        initialHeading = (storedHeading + Math.PI) % (2 * Math.PI);
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
    
    public double getNgSignChangesPerCycleUpThreshold() {
        return ngSignChangesPerCycleUpThreshold;
    }
    
    public void setNgSignChangesPerCycleUpThreshold(double t) {
        ngSignChangesPerCycleUpThreshold = t;
    }

    public double getNgSignChangesPerCycleDownThreshold() {
        return ngSignChangesPerCycleDownThreshold;
    }

    public void setNgSignChangesPerCycleDownThreshold(double t) {
        ngSignChangesPerCycleDownThreshold = t;
    }

//    public void followLine() {
//        double[][] readings = getLineData(); //the array stores the readings on the button pusher side as the front
//
//        String maxReadingIndexes = Utils.findTwoMaxIndexesAsString(readings);
//
//
//        if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(frontRight)) {
//            wheels.drive(-0.3, 0, readings[0][0] > readings[0][1] ? -0.1 : 0.1  , false);
//        } else if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(backRight)) {
//            wheels.drive(-0.3, 0, -0.2, false);
//        } else if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(backLeft)) {
//            wheels.drive(-0.3, -0.2, 0, false);
//        } else if (maxReadingIndexes.contains(frontRight) && maxReadingIndexes.contains(backLeft)) {
//            wheels.drive(-0.3, 0, 0.2, false);
//        } else if (maxReadingIndexes.contains(frontRight) && maxReadingIndexes.contains(backRight)) {
//            wheels.drive(-0.3, 0.2, 0, false);
//        } else if (maxReadingIndexes.contains(backLeft) && maxReadingIndexes.contains(backRight)) {
//            wheels.drive(-0.3, 0, readings[0][0] > readings[0][1] ? -0.1 : 0.1, false);
//        }
//    }

    public void findBeaconButton(boolean isRed) {

        boolean goingForward = false;


        double previousReading = getBeaconColor()[isRed ? 1: 0],
                currentReading,
                maxColor = 0,
                directionSwitches = 0;


        while (true) {

            if ((currentReading = getBeaconColor()[isRed ? 1: 0]) >  maxColor)
                maxColor = currentReading;

            if (directionSwitches > 3 && currentReading == maxColor) {
                wheels.stop();
                return;
            }

            if (currentReading < previousReading) {
                goingForward = !goingForward;
                directionSwitches++;
            }

            if (directionSwitches > 6) { //we got an outlier maximum and can't recreate it, lower the threshold`
                maxColor--;
                directionSwitches = 0;
            }

            compensatedTranslate(Math.PI / 2 * (goingForward ? 1 : 3), false, true);

            previousReading = currentReading;
        }

    }



}
