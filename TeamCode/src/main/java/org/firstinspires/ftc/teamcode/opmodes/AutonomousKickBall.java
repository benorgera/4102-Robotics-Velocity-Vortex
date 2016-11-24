package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/5/16.
 */

@Autonomous(name="Kick Ball", group="4102")
public class AutonomousKickBall extends LinearOpMode {

    private Wheels wheels;

    @Override
    public void runOpMode() {
        Hardware.setMap(hardwareMap);
        wheels = Hardware.getWheels();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the driver station to

        Utils.sleep(10000);

        driveByTime(3000, 0, 0.5, 0);

    }

    private void driveByTime(long ms, double xVel, double yVel, double angularVel) {
        wheels.drive(xVel, yVel, angularVel, false);
        Utils.sleep(ms);
        wheels.stop();
    }
}
