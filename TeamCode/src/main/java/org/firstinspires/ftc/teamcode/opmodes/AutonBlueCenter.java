package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 11/24/16.
 */

@Autonomous(name = "Blue [  ]", group = "4102")
public class AutonBlueCenter extends LinearOpMode {

    private AutonomousImplementation a; //uses an Autonomous Implementation class
    
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap, this, true, false, telemetry); //initializes hardware

        telemetry.update();

        a = new AutonomousImplementation(false, true); //new Autonomous Implementation with isRed false and isParkingCenter true

        waitForStart(); //wait for start to be pressed

        try {
            a.run(); //runs
        } catch (Exception e) {
            Hardware.print("Exception: " + e.getMessage()); //prints exceptions
        } finally {
            Hardware.freezeAllMotorFunctions(); //stops motors at the end
        }
    }
}
