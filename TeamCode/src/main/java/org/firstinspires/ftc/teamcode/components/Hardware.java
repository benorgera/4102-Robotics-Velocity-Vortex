package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
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
                map.get(BNO055IMU.class, "imu"),
                map.servo.get("imu-arm"),
                map.servo.get("imu-latch"),
                new ColorSensor[][] {
                        {
                                map.colorSensor.get("front-left-color-sensor"),
                                map.colorSensor.get("front-right-color-sensor")
                        },
                        {
                                map.colorSensor.get("back-left-color-sensor"),
                                map.colorSensor.get("back-right-color-sensor")
                        }
                },
                (ModernRoboticsAnalogOpticalDistanceSensor) map.opticalDistanceSensor.get("ods"),
                (ModernRoboticsI2cColorSensor) map.colorSensor.get("beacon-color-sensor")
        );

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
