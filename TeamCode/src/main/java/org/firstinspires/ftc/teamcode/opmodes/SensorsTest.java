package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

import java.util.Arrays;

/**
 * Created by benorgera on 12/1/16.
 */

@TeleOp (name = "Sensors Test", group = "4102")
public class SensorsTest extends LinearOpMode {


    private boolean isUppingNgConstant = false;
    private boolean isDroppingNgConstant = false;

    private boolean isUppingStrafeConstant = false;
    private boolean isDroppingStrafeConstant = false;

    private Sensors sensors;

    private Intake intake;
    private Wheels wheels;

    private final double translateSpeed = 0.3          ;

    private double strafeConstant = 1.5;

    private double ngCompensationConstant = -0.6;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.setMap(hardwareMap);

        intake = Hardware.getIntake();
        wheels = Hardware.getWheels();
        sensors = Hardware.getSensors();

        sensors.centerIMU();
        sensors.resetHeading();

        waitForStart();

        sensors.initImu();

        while (opModeIsActive()) {

            if (!gamepad1.dpad_down && ! gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad2.dpad_down && ! gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up)
                wheels.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, false);

            if (gamepad1.a) sensors.centerIMU();

            if (gamepad1.b) sensors.foldIMU();

            if (gamepad1.x) sensors.resetHeading();

            if (gamepad1.left_bumper) sensors.resetIntegrator();

            if (gamepad1.dpad_up) translate(Math.PI / 2);

            if (gamepad1.dpad_down) translate(3 * Math.PI / 2);

            if (gamepad1.dpad_right) translate(0);

            if (gamepad1.dpad_left) translate(Math.PI);

            if (gamepad1.right_bumper) wheels.drive(0, 0, 1, false);

            if (gamepad1.y || gamepad2.y) {
                wheels.stop();
                intake.stop();
            }

            if (gamepad2.dpad_right) translate(Math.PI / 4);

            if (gamepad2.dpad_up) translate(3 * Math.PI / 4);

            if (gamepad2.dpad_left) translate(5 * Math.PI / 4);

            if (gamepad2.dpad_down) translate(7 * Math.PI/ 4);


            if (gamepad2.left_bumper && !isDroppingNgConstant) ngCompensationConstant -= 0.01;

            isDroppingNgConstant = gamepad2.left_bumper;


            if (gamepad2.right_bumper && !isUppingNgConstant) ngCompensationConstant += 0.01;

            isUppingNgConstant = gamepad2.right_bumper;


            if (gamepad2.a && !isUppingStrafeConstant)
                strafeConstant += 0.01;

            isUppingStrafeConstant = gamepad2.a;

            if (gamepad2.b && !isDroppingStrafeConstant)
                strafeConstant -= 0.01;

            isDroppingStrafeConstant = gamepad2.b;

            intake.run(gamepad2.right_stick_x);

            telemetry.addData("rH | iH | head", Utils.toString(Utils.toDegrees(sensors.getRawHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getInitialHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getHeading())));
            telemetry.addData("ng | strafe", Utils.toString(ngCompensationConstant) + " | " + Utils.toString(strafeConstant));
            telemetry.addData("optical distance", Utils.toString(sensors.getOpticalDistance()));
            telemetry.addData("color", "blue: " + Utils.toString(sensors.getBeaconColor()[0]) + ", red: " + Utils.toString(sensors.getBeaconColor()[1]));
            telemetry.addData("line readings", Utils.toString(sensors.getLineData()[0][0]) + " " + Utils.toString(sensors.getLineData()[0][1]) + " " + Utils.toString(sensors.getLineData()[1][0]) + " " + Utils.toString(sensors.getLineData()[1][1]));

            telemetry.update();
        }
    }


    private void translate(double thetaDesired) { //translate the robot at desired angle [0, 2π], and compensate for unwanted rotation/drift our orientation and estimated position
        double power = translateSpeed * (1 + strafeConstant * Math.abs(Math.cos(thetaDesired))),
                ngVel = Utils.trim(-1, 1, sensors.getHeading() * ngCompensationConstant); //compensate for rotation by accounting for change in heading
//            thetaActual = Utils.atan3(sensors.getPosition().y, sensors.getPosition().x), //the direction of displacement thus far
//                thetaDif = thetaDesired - thetaActual, //the difference between desired and actual displacement direction
//                thetaNew = thetaDesired + thetaDif * k; //compensate for drift by accounting for the difference in desired and actual direction

        telemetry.addData("ngVel Compensation", ngVel);

        wheels.drive(Math.cos(thetaDesired) * power, Math.sin(thetaDesired) * power, ngVel, false);
    }
}
