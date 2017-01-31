package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.components.Hardware;

/**
 * Created by benorgera on 11/24/16.
 */

@Autonomous(name = "Auton Blue", group = "4102")
public class AutonBlue extends LinearOpMode {

    private AutonomousImplementation a;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap, this, true, telemetry);

        telemetry.update();

        a = new AutonomousImplementation(false);

        waitForStart();

        a.run();

        Hardware.freezeAllMotorFunctions();
    }
}
