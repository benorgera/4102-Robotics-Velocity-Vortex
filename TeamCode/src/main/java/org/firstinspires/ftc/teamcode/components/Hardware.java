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

    private static boolean isAuton = false;

    private static Lift lift;
    private static Wheels wheels;
    private static Sensors sensors;
    private static Shooter shooter;
    private static Intake intake;

    public static void setIsAuton(boolean a) {
        isAuton = a;
    }

    public static void setMap(HardwareMap m) {
        clean();
        map = m;
    }

    public static Wheels getWheels() {
        return wheels == null ? wheels = new Wheels(
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
        ) : wheels;
    }

    public static Sensors getSensors() {
        return sensors == null ? sensors = new Sensors(
                map.get(BNO055IMU.class, "imu"),
                map.colorSensor.get("left-color-sensor"),
                map.colorSensor.get("right-color-sensor"),
                (ModernRoboticsAnalogOpticalDistanceSensor) map.opticalDistanceSensor.get("ods"),
                (ModernRoboticsI2cColorSensor) map.colorSensor.get("beacon-color-sensor")
        ) : sensors;

    }

    public static Shooter getShooter() {
        return shooter == null ? shooter = new Shooter(
                new DcMotor[] {
                        map.dcMotor.get("shooter-left"),
                        map.dcMotor.get("shooter-right")
                },
                map.servo.get("door")
        ) : shooter;
    }

    public static Intake getIntake() {
        return intake == null ? intake = new Intake(map.dcMotor.get("intake"), map.servo.get("ramp"), isAuton) : intake;
    }

    public static Lift getLift() {
        return lift == null ? lift = new Lift(map.dcMotor.get("lift"), map.servo.get("latch")) : lift;
    }

    public static void freezeAllMotorFunctions() {
        if (shooter != null)
            shooter.stop();

        if (intake != null)
            intake.stopElevator();

        if (wheels != null)
            wheels.stop();

        if (lift != null)
            lift.stop();
    }

    private static void clean() {
        map = null;
        lift = null;
        wheels = null;
        sensors = null;
        shooter = null;
        intake = null;
    }
}
