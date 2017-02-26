package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/5/16.
 */

@Autonomous(name = "Auton Shoot", group = "4102")
public class AutonShootRed extends LinearOpMode {

    private Wheels wheels;

    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, true, telemetry); //initializes hardware
        Hardware.getLift();
        Hardware.getShooter();
        wheels = Hardware.getWheels();
        
        telemetry.addData("4102", "Let's kick up");
        telemetry.update();
        
        waitForStart(); //wait for the start button to be pressed
        
        Hardware.getShooter().shoot(7.3, true); //makes a shot
        
        Hardware.sleep(10000); //sleeps for 10 seconds
        
        driveByTime(2000, 0, -0.5, 0); //drives towards the cap ball to knock it over
    }
    
    private void driveByTime(long ms, double xVel, double yVel, double angularVel) { //allows us to drive for a certain amount of time
        wheels.drive(xVel, yVel, angularVel, false);
        Hardware.sleep(ms);
        wheels.stop();
    }
}-
