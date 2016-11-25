package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousBlue extends LinearOpMode {

    private AutonomousImplementation a;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.setMap(hardwareMap);

        a = new AutonomousImplementation(false);

        waitForStart();

        while (opModeIsActive() && a.isActive()) a.loop();

        a.stop();

    }
}
