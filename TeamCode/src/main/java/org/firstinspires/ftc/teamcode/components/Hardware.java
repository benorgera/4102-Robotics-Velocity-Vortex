package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousImplementation;

/**
 * Created by benorgera on 11/24/16.
 */

public class Hardware {

    private static HardwareMap map;

    public static void setMap(HardwareMap m) {
        map = m;
    }

    public static Wheels getWheels() {
        return new Wheels(
                new DcMotor[][] {
                        {
                                map.dcMotor.get("front-left-wheel"),
                                map.dcMotor.get("front-right-wheel")
                        },
                        {
                                map.dcMotor.get("back-left-wheel"),
                                map.dcMotor.get("back-right-wheel")
                        }
                }
        );
    }

    public static Sensors getSensors() {
        return new Sensors(
                (ModernRoboticsI2cGyro) map.gyroSensor.get("gyro"),
                map.servo.get("gyro-arm"),
                new ColorSensor[][] {
                        {
                                map.colorSensor.get("front-left-sensor"),
                                map.colorSensor.get("front-right-sensor")
                        },
                        {
                                map.colorSensor.get("back-left-sensor"),
                                map.colorSensor.get("back-right-sensor")
                        }
                },
                (ModernRoboticsAnalogOpticalDistanceSensor) map.opticalDistanceSensor.get("ods"),
                (ModernRoboticsI2cColorSensor) map.colorSensor.get("beacon-sensor")
        );



        //    new AdafruitBNO055IMU(hardwareMap.i2cDeviceSynch.get("imu"));
    }

    public static Shooter getShooter() {
        return new Shooter(
                new DcMotor[] {
                        map.dcMotor.get("shooter-left"),
                        map.dcMotor.get("shooter-right")
                }
        );
    }

    public static Intake getIntake() {
        return new Intake(map.dcMotor.get("intake"));
    }

}
