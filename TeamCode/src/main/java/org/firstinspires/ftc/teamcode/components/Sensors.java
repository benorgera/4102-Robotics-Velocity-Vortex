package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by benorgera on 10/25/16.
 */
public class Sensors {

    private BNO055IMU imu;
    private Servo gyroArm;
    private Servo latchServo;
    private ColorSensor[][] lineSensors;
    private ModernRoboticsAnalogOpticalDistanceSensor ods;
    private ModernRoboticsI2cColorSensor beaconSensor;

    private double initialHeading = 0;

    private final double[] latchPositions = new double[] {0, 0.7}; //gyro latch positions (latched and unlatched respectively)
    private final double[] gyroPositions = new double[] {0.2, 1}; //gyro positions (centered and folded respectively)

    public Sensors(BNO055IMU imu, Servo gyroArm, Servo latchServo, ColorSensor[][] lineSensors, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor) {
        this.imu = imu;
        this.gyroArm = gyroArm;
        this.latchServo = latchServo;
        this.lineSensors = lineSensors;
        this.ods = ods;
        this.beaconSensor = beaconSensor;

        foldIMU(); //fold imu out of shooter's path upon initialization
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
//        parameters.accelerationIntegrationAlgorithm = new Integrator();

        imu.initialize(parameters);

        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, 0x24);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, 0x03);

        resetHeading();
    }

    public void integrateAcceleration() { //start integrating linear acceleration to find position and velocity
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void stopIntegratingAcceleration() { //stop integrating linear acceleration
        imu.stopAccelerationIntegration();
    }

    public double[][] getLineData() { //get a 2d array representing the readings (where the front is the side with the button pusher)
        double[][] res = new double[2][2];

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
            res[i][j] = lineSensors[i][j].green();

        return res;
    }

    public void centerIMU() { //swing the imu in to the center of the robot
        gyroArm.setPosition(gyroPositions[0]);
        Utils.sleep(1000);
        latchServo.setPosition(latchPositions[0]);
    }

    public void foldIMU() { //swing the imu out of the center of the robot
        latchServo.setPosition(latchPositions[1]);
        gyroArm.setPosition(gyroPositions[1]);
    }

    public double getHeading() { // [-π, π]
        return Utils.angleDifference(getRawHeading(), initialHeading);
    }

    public void resetHeading() {
        initialHeading = getRawHeading();
    }

    private double getRawHeading() {
        return imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).toAxesReference(AxesReference.INTRINSIC).thirdAngle;
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
}
