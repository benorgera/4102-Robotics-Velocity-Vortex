package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/5/16.
 */

@Autonomous(name = "Auton Shoot Red", group = "4102")
public class AutonShootRed extends LinearOpMode {

    private Wheels wheels;

    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, true, telemetry);
        Hardware.getLift();
        Hardware.getShooter();
        wheels = Hardware.getWheels();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the driver station to

        Hardware.getShooter().shoot(6);
        Hardware.sleep(6500);

        driveByTime(2000, 0, -0.5, 0);
        driveByTime(1000, 0, 0, -1);
    }

    private void driveByTime(long ms, double xVel, double yVel, double angularVel) {
        wheels.drive(xVel, yVel, angularVel, false);
        Hardware.sleep(ms);
        wheels.stop();
    }
}
