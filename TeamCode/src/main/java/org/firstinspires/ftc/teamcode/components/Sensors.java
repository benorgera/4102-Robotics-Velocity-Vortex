package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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

    private AdafruitBNO055IMU imu;
    private Servo gyroArm;
    private ColorSensor[][] lineSensors;
    private ModernRoboticsAnalogOpticalDistanceSensor ods;
    private ModernRoboticsI2cColorSensor beaconSensor;

    private final double[] gyroPositions = new double[] {0, 0.5}; //gyro positions (centered and folded respectively)

    public Sensors(AdafruitBNO055IMU imu, Servo gyroArm, ColorSensor[][] lineSensors, ModernRoboticsAnalogOpticalDistanceSensor ods, ModernRoboticsI2cColorSensor beaconSensor) {
        this.imu = imu;
        this.gyroArm = gyroArm;
        this.lineSensors = lineSensors;
        this.ods = ods;
        this.beaconSensor = beaconSensor;
    }

    public void initImu() {

        BNO055IMU.Parameters p = new BNO055IMU.Parameters();

        p.mode = BNO055IMU.SensorMode.NDOF; //ndof and imu are the two fusion modes, but ndof uses magnetometer too
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        p.pitchMode = BNO055IMU.PitchMode.ANDROID; //counterclockwise rotation is positive
        p.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;

        p.loggingEnabled = true;
        p.loggingTag = "IMU";

        p.accelerationIntegrationAlgorithm = new Integrator();

        //        p.calibrationData
        //        p.calibrationDataFile

        imu.initialize(p);

        while (imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION) Utils.sleep(10); //wait for initialization to complete
    }

    public void integrateAcceleration(boolean shouldIntegrate) { //start or stop integrating the linear acceleration of the imu
        if (shouldIntegrate)
            imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        else
            imu.stopAccelerationIntegration();
    }


    public double[][] getLineData() { //get a 2d array representing the readins
        double[][] res = new double[2][2];

        for (int i = 0; i < 2; i++) for (int j = 0; j < 2; i++)
            res[i][j] = lineSensors[i][j].green();

        return res;
    }

    public void centerIMU() { //swing the imu in to the center of the robot
        gyroArm.setPosition(gyroPositions[0]);
    }

    public void foldIMU() { //swing the imu out of the center of the robot
        gyroArm.setPosition(gyroPositions[1]);
    }

}
