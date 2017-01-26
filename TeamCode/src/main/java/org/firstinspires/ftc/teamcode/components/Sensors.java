package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    private LinearOpMode opMode;

    private double initialHeading = 0;

    private final double compensatedTranslateSpeed = 0.2;

    private final double headingAccuracyThreshold = 1 * Math.PI / 180; //1.5 degrees

    private double ngSignChangesPerCycleUpThreshold = 0.001; //the low number of sign changes per cycle needed to cause an ngConstant increase
    private double ngSignChangesPerCycleDownThreshold = 0.005; //the high number of sign changes per cycle needed to cause an ngConstant decrease

    private double strafeConstant = 1.71;
    private double ngConstant = 0.6;

    public Sensors(BNO055IMU imu, ColorSensor leftColorSensor, ColorSensor rightColorSensor, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor) {
        this.imu = imu;
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

//        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
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

    public String compensatedTranslate(double thetaDesired, boolean isExtraSlow) { //translate robot with rotation compensation (must be called on a loop)
        double power = compensatedTranslateSpeed * (1 + strafeConstant * Math.abs(Math.cos(thetaDesired))) * (isExtraSlow ? 0.48 : 1),
                ngVel = Utils.trim(-1, 1, -1 * getHeading() * ngConstant); //compensate for rotation by accounting for change in heading

        wheels.drive(Math.cos(thetaDesired) * power, Math.sin(thetaDesired) * power, ngVel, false);

        return "" + ngVel;
    }

    public void centerOnZero() {
        double heading = getHeading();
        while (Math.abs(heading) > headingAccuracyThreshold && opMode.opModeIsActive()) {
            heading = getHeading();

            wheels.drive(0, 0, Utils.trim(0.05, 0.08, .04 * heading) * (heading > 0 ? -1 : 1), false);
            Utils.sleep(15);
            wheels.stop();
            Utils.sleep(70);
        }
        wheels.stop();
    }

    public void turnAround() { //must call center on zero after
        double storedHeading = initialHeading,
                heading;

        resetHeading();

        while ((heading = getHeading()) < Math.PI || heading < 0) {
            wheels.drive(0, 0, -0.08, false);
        }

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

    public double[] getLineReadings() {
        return new double[] {
                leftColorSensor.green(), //green is a measure of white
                rightColorSensor.green()
        };
    }

    private void followLine() {
        double[] readings = getLineReadings();
        double left = readings[0],
                right = readings[1];

        if (Math.abs(left - right) <= 2) { //both sensors equally on the white line
            compensatedTranslate(0, true);
        } else { //left more on the white line turn left, right turn right
            compensatedTranslate(Math.PI / 4 * (left > right ? 1 : -1), true);
        }
    }

    public void findBeaconButton(boolean isRed) {

        boolean goingForward = false;


        double previousReading = getBeaconColor()[isRed ? 1 : 0],
                currentReading,
                maxColor = 0,
                directionSwitches = 0;


        while (opMode.opModeIsActive()) {

            if ((currentReading = getBeaconColor()[isRed ? 1 : 0]) >  maxColor)
                maxColor = currentReading;

            if (directionSwitches > 3 && currentReading == maxColor) {
                wheels.stop();
                return;
            }

            if (currentReading < previousReading) {
                goingForward = !goingForward;
                directionSwitches++;
            }

            if (directionSwitches > 6) { //we got an outlier maximum and can't recreate it, lower the threshold
                maxColor--;
                directionSwitches = 0;
            }

            compensatedTranslate(Math.PI / 2 * (goingForward ? 1 : 3), true);

            previousReading = currentReading;
        }

    }

    public void driveUntilOdsThreshold(double theta, double odsThreshold, boolean isMax) {
        //drive until we reach the beacon
        wheels.readySoftStart(500);
        while (opMode.opModeIsActive() && isMax ? (getOpticalDistance() < odsThreshold) : (getOpticalDistance() > odsThreshold))
            compensatedTranslate(theta, true);
        wheels.softStop(500);
    }

    public void followLineUntilOdsThreshold(double odsThreshold) {
        //drive until we reach the beacon
        wheels.readySoftStart(500);
        while (opMode.opModeIsActive() && getOpticalDistance() < odsThreshold)
            followLine();
        wheels.softStop(500);
    }

    public void driveUntilLineReadingThreshold(double theta, double whiteLineReadingThreshold) {
        wheels.readySoftStart(500);
        while (opMode.opModeIsActive() && Utils.getMaxMagnitude(getLineReadings()) < whiteLineReadingThreshold)
            compensatedTranslate(theta, false);
        wheels.softStop(500);
    }



}
