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
        Hardware.init(hardwareMap, this, true, telemetry);

        a = new AutonomousImplementation(true, true);

        waitForStart();

        try {
            a.run();
        } catch (Exception e) {
            Hardware.print("Exception: " + e.getMessage());
        } finally {
            Hardware.freezeAllMotorFunctions();
        }
    }
}
