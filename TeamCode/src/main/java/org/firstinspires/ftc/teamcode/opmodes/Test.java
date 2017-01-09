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

@TeleOp (name = "Test", group = "4102")
public class Test extends LinearOpMode {

    private boolean gamePad1AState = false;
    private boolean gamePad1BState = false;

    private boolean isUppingNgConstant = false;
    private boolean isDroppingNgConstant = false;

    private boolean isUppingStrafeConstant = false;
    private boolean isDroppingStrafeConstant = false;

    private boolean isDroppingNgConstantRateUpThreshold = false;
    private boolean isUppingNgConstantRateUpThreshold = false;

    private boolean isDroppingNgConstantRateDownThreshold = false;
    private boolean isUppingNgConstantRateDownThreshold = false;


    private boolean gamePad1DpadUpState = false;
    private boolean gamePad1DpadDownState = false;
    private boolean gamePad1DpadRightState = false;
    private boolean gamePad1DpadLeftState = false;

    private boolean gamePad2DpadUpState = false;
    private boolean gamePad2DpadDownState = false;
    private boolean gamePad2DpadRightState = false;
    private boolean gamePad2DpadLeftState = false;

    private Sensors sensors;

    private Intake intake;
    private Wheels wheels;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.setMap(hardwareMap);

        wheels = Hardware.getWheels();
        intake = Hardware.getIntake();
        sensors = Hardware.getSensors();

        sensors.resetHeading();

        waitForStart();

        sensors.initImu();

        while (opModeIsActive()) {

            if (gamepad1.a && !gamePad1AState) {
                intake.startIntaking();
            }

            gamePad1AState = gamepad1.a;



            if (gamepad1.a && !gamePad1BState) {
                intake.stopIntaking();
            }

            gamePad1BState = gamepad1.b;
            
            //drive if no autonomous driving is occurring
            if (!gamepad1.dpad_down && ! gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad2.dpad_down && ! gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up && (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0))
                wheels.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, false);

            if (gamepad1.b) sensors.findBeaconButton(gamepad1.y, this);

            if (gamepad1.x) sensors.resetHeading();


            if (gamepad1.dpad_up) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp | cyc | rate", sensors.compensatedTranslate(Math.PI / 2, !gamePad1DpadUpState, false));
            } else if (gamePad1DpadUpState) {
                wheels.softStop();
            }

            gamePad1DpadUpState = gamepad1.dpad_up;

            if (gamepad1.dpad_down) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp | cyc | rate", sensors.compensatedTranslate(3 * Math.PI / 2, !gamePad1DpadDownState, false));
            } else if (gamePad1DpadDownState) {
                wheels.softStop();
            }

            gamePad1DpadDownState = gamepad1.dpad_down;

            if (gamepad1.dpad_right) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(0, !gamePad1DpadRightState, false));
            } else if (gamePad1DpadRightState) {
                wheels.softStop();
            }

            gamePad1DpadRightState = gamepad1.dpad_right;

            if (gamepad1.dpad_left) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(Math.PI, !gamePad1DpadLeftState, false));
            } else if (gamePad1DpadLeftState) {
                wheels.softStop();
            }

            gamePad1DpadLeftState = gamepad1.dpad_left;

            if (gamepad2.dpad_right) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(Math.PI / 4, !gamePad2DpadRightState, false));
            } else if (gamePad2DpadRightState) {
                wheels.softStop();
            }

            gamePad2DpadRightState = gamepad2.dpad_right;

            if (gamepad2.dpad_up) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(3 * Math.PI / 4, !gamePad2DpadUpState, false));
            } else if (gamePad2DpadUpState) {
                wheels.softStop();
            }

            gamePad2DpadUpState = gamepad2.dpad_up;

            if (gamepad2.dpad_left) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(5 * Math.PI / 4, !gamePad2DpadLeftState, false));
            } else if (gamePad2DpadLeftState) {
                wheels.softStop();
            }

            gamePad2DpadLeftState = gamepad2.dpad_left;

            if (gamepad2.dpad_down) {
                sensors.resetCompensatedTranslate();
                telemetry.addData("ngComp", sensors.compensatedTranslate(7 * Math.PI/ 4, !gamePad2DpadDownState, false));
            } else if (gamePad2DpadDownState) {
                wheels.softStop();
            }

            gamePad2DpadDownState = gamepad2.dpad_down;


            if (gamepad2.right_bumper && !isUppingNgConstant)
                sensors.setNgConstant(sensors.getNgConstant() + 0.001);

            isUppingNgConstant = gamepad2.right_bumper;

            if (gamepad2.left_bumper && !isDroppingNgConstant)
                sensors.setNgConstant(sensors.getNgConstant() - 0.01);
            
            isDroppingNgConstant = gamepad2.left_bumper;
            
            if (gamepad2.start && !isUppingStrafeConstant)
                sensors.setStrafeConstant(sensors.getStrafeConstant() + 0.01);

            isUppingStrafeConstant = gamepad2.start;

            if (gamepad2.back && !isDroppingStrafeConstant)
                sensors.setStrafeConstant(sensors.getStrafeConstant() - 0.01);

            isDroppingStrafeConstant = gamepad2.back;


            if (gamepad1.right_bumper && !isUppingNgConstantRateUpThreshold)
                sensors.setNgSignChangesPerCycleUpThreshold(sensors.getNgSignChangesPerCycleUpThreshold() + 0.001);

            isUppingNgConstantRateUpThreshold = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !isDroppingNgConstantRateUpThreshold)
                sensors.setNgSignChangesPerCycleUpThreshold(sensors.getNgSignChangesPerCycleUpThreshold() - 0.001);

            isDroppingNgConstantRateUpThreshold = gamepad1.left_bumper;

            if (gamepad1.start && !isUppingNgConstantRateDownThreshold)
                sensors.setNgSignChangesPerCycleDownThreshold(sensors.getNgSignChangesPerCycleDownThreshold() + 0.001);

            isUppingNgConstantRateDownThreshold = gamepad1.start;

            if (gamepad1.back && !isDroppingNgConstantRateDownThreshold)
                sensors.setNgSignChangesPerCycleDownThreshold(sensors.getNgSignChangesPerCycleDownThreshold() - 0.001);

            isDroppingNgConstantRateDownThreshold = gamepad1.back;

            if (gamepad2.b)
                sensors.turnAround();

            if (gamepad2.x) sensors.centerOnZero(this);

            telemetry.addData("rH | iH | head", Utils.toString(Utils.toDegrees(sensors.getRawHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getInitialHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getHeading())));
            telemetry.addData("ng | strafe", Utils.toString(sensors.getNgConstant()) + " | " + Utils.toString(sensors.getStrafeConstant()));
            telemetry.addData("ngRateUp | ngRateDown", sensors.getNgSignChangesPerCycleUpThreshold() + " | " + sensors.getNgSignChangesPerCycleDownThreshold());
            telemetry.addData("blue | red | distance",  Utils.toString(sensors.getBeaconColor()[0]) + " | " + Utils.toString(sensors.getBeaconColor()[1]) + Utils.toString(sensors.getOpticalDistance()));
//            telemetry.addData("line readings", Utils.toString(sensors.getLineData()[0][0]) + " " + Utils.toString(sensors.getLineData()[0][1]) + " " + Utils.toString(sensors.getLineData()[1][0]) + " " + Utils.toString(sensors.getLineData()[1][1]));

            telemetry.update();
        }
    }
}
