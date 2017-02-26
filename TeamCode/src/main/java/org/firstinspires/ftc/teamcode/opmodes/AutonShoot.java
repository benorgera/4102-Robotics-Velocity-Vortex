package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Wheels;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by benorgera on 11/5/16.
 */

@Autonomous(name = "Auton Shoot", group = "4102")
public class AutonShoot extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, true, telemetry); //initializes hardware
        Hardware.getLift();
        Hardware.getShooter();
        Hardware.getWheels();
        
        telemetry.addData("4102", "Let's kick up");
        telemetry.update();
        
        waitForStart(); //wait for the start button to be pressed
        
        Hardware.getShooter().prepShot(7.1); //preps a shot
        Hardware.getSensors().driveByTime(-Math.PI / 2, 200, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall
        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(1000); //allows shooter to get to the correct speed
        Hardware.getShooter().shoot(7.1, true); //shoots balls
        
        Hardware.sleep(9000); //sleeps for 9 seconds
        
        Hardware.getSensors().driveByTime(-Math.PI / 2, 2000, true, 0.5); //drives towards the cap ball to knock it over
    }
}
