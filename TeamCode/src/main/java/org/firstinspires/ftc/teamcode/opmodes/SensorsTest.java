package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Utils;

import java.util.Arrays;

/**
 * Created by benorgera on 12/1/16.
 */

@TeleOp (name = "Sensors Test", group = "4102")
public class SensorsTest extends LinearOpMode {


    private Sensors sensors;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.setMap(hardwareMap);

        sensors = Hardware.getSensors();

        sensors.centerIMU();
        sensors.resetHeading();
        sensors.integrateAcceleration();

        waitForStart();

        sensors.initImu();

        while (opModeIsActive()) {
            if (gamepad1.a) sensors.centerIMU();

            if (gamepad1.b) sensors.foldIMU();

            if (gamepad1.x) sensors.resetHeading();

            if (gamepad1.left_bumper) sensors.integrateAcceleration();

            if (gamepad1.right_bumper) sensors.stopIntegratingAcceleration();

            telemetry.addData("heading", sensors.getHeading() * 180 / Math.PI);
            telemetry.addData("velocity", sensors.getVelocity());
            telemetry.addData("position", sensors.getPosition());
            telemetry.addData("optical distance", sensors.getOpticalDistance());
            telemetry.addData("color", "blue: " + Utils.toString(sensors.getBeaconColor()[0]) + ", red: " + Utils.toString(sensors.getBeaconColor()[1]));
            telemetry.addData("line readings", Utils.toString(sensors.getLineData()[0][0]) + " " + Utils.toString(sensors.getLineData()[0][1]) + " " + Utils.toString(sensors.getLineData()[1][0]) + " " + Utils.toString(sensors.getLineData()[1][1]));

            telemetry.update();

        }
    }
}
