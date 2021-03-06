package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbI2cController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.I2cController;

/**
 * Created by benorgera on 11/5/16.
 */

@Autonomous(name = "Shoot No Park", group = "4102")
public class AutonShootNoPark extends LinearOpMode {


    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, true, false, telemetry); //initializes hardware
        Hardware.getLift();
        Hardware.getShooter();
        Hardware.getWheels();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the start button to be pressed

        Hardware.getShooter().prepShot(6.6); //preps a shot
        Hardware.getSensors().driveByTime(-Math.PI / 2, 600, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall
        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(3000); //allows shooter to get to the correct speed
        Hardware.getShooter().shoot(1000); //shoots balls
    }
}
