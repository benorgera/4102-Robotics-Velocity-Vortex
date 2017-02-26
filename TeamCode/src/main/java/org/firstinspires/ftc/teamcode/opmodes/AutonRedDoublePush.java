package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 11/24/16.
 */

@Autonomous(name = "Auton Red Double Push", group = "4102")
public class AutonRedDoublePush extends LinearOpMode {

    private AutonomousImplementation a;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap, this, true, telemetry); //initializes hardware
        
        a = new AutonomousImplementation(true, true); //new Autonomous Implementation with isRed being true and isDoublePush being true
        
        waitForStart(); //wait for start to be pressed
        
        try {
            a.run(); //runs
        } catch (Exception e) {
            Hardware.print("Exception: " + e.getMessage()); //prints exceptions
        } finally {
            Hardware.freezeAllMotorFunctions(); //freezes motors at the end
        }
    }
}
