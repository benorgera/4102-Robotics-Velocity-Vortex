package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/24/16.
 */

public class Hardware {

    private static HardwareMap map;

    private static boolean isAuton = false;

    private static double gyroConstant = 1;

    private static LinearOpMode opMode;
    private static Lift lift;
    private static Wheels wheels;
    private static Sensors sensors;
    private static Shooter shooter;
    private static Intake intake;
    private static Telemetry t;
    private static String output = "\n";

    public static void sleep(long ms) {
        opMode.sleep(ms);
    }

    public static void init(HardwareMap map, LinearOpMode opMode, boolean isAuton, Telemetry t) {
        clean();
        Hardware.opMode = opMode;
        Hardware.isAuton = isAuton;
        Hardware.map = map;
        Hardware.t = t;
    }

    public static void print(String s) {
        if (t == null || s.isEmpty())
            return;

        output += "\n" + s;

        t.addData("H", output);
        t.update();
    }

    public static void clearLog() {
        output = "\n";
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
                (ModernRoboticsAnalogOpticalDistanceSensor) map.opticalDistanceSensor.get("ods-left"),
                (ModernRoboticsAnalogOpticalDistanceSensor) map.opticalDistanceSensor.get("ods-right"),
                (ModernRoboticsI2cColorSensor) map.colorSensor.get("beacon-color-sensor"),
                map.voltageSensor.iterator().next()
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

    public static boolean active() {
        return opMode.opModeIsActive() && !opMode.isStopRequested();
    }

    private static void clean() {
        output = null;
        map = null;
        lift = null;
        wheels = null;
        sensors = null;
        shooter = null;
        intake = null;
        opMode = null;
        t = null;
        isAuton = false;
    }

    public static void setGyroConstant(double d) {
        gyroConstant = d;
    }

    public static double getGyroConstant() {
        return gyroConstant;
    }
}
