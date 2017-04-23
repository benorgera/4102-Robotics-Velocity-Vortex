package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 11/24/16.
 */

@Autonomous(name = "Red [  ]", group = "4102")
public class AutonRedCenter extends LinearOpMode {

    private AutonomousImplementation a; //uses an Autonomous Implementation class

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap, this, true, telemetry); //initializes hardware

        a = new AutonomousImplementation(true, true); //new Autonomous Implementation with isRed true and isParkingCenter true

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
