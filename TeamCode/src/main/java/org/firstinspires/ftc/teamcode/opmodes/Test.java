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

    private boolean gamePad2XState = false;
    private boolean gamepad2YState = false;

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

    private Wheels wheels;

    private long softMotionTime = 500;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap, this, false);

        wheels = Hardware.getWheels();
        sensors = Hardware.getSensors();

        sensors.resetHeading();

        waitForStart();

        sensors.initImu();

        while (opModeIsActive()) {


//            if (gamepad1.b)
//                wheels.readySoftStart(softMotionTime);

            if (gamepad2.x && ! gamePad2XState)
                softMotionTime += 50;

            gamePad2XState = gamepad1.a;

            if (gamepad2.y && !gamepad2YState)
                    softMotionTime -= 50;


            if (gamepad1.b) wheels.stop();

            //drive if no autonomous driving is occurring
            if (!gamepad1.dpad_down && ! gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad2.dpad_down && ! gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up) {
                if ((Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0))
                    wheels.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, false);
            }

            if (gamepad2.a) sensors.findBeaconButton(gamepad1.y);

            if (gamepad1.x) sensors.resetHeading();


            if (gamepad1.dpad_up) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(Math.PI / 2, false));
            } else if (gamePad1DpadUpState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadUpState = gamepad1.dpad_up;

            if (gamepad1.dpad_down) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(3 * Math.PI / 2, false));
            } else if (gamePad1DpadDownState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadDownState = gamepad1.dpad_down;

            if (gamepad1.dpad_right) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(0, false));
            } else if (gamePad1DpadRightState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadRightState = gamepad1.dpad_right;

            if (gamepad1.dpad_left) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(Math.PI, false));
            } else if (gamePad1DpadLeftState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadLeftState = gamepad1.dpad_left;

            if (gamepad2.dpad_right) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(Math.PI / 4, false));
            } else if (gamePad2DpadRightState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadRightState = gamepad2.dpad_right;

            if (gamepad2.dpad_up) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(3 * Math.PI / 4, false));
            } else if (gamePad2DpadUpState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadUpState = gamepad2.dpad_up;

            if (gamepad2.dpad_left) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(5 * Math.PI / 4, false));
            } else if (gamePad2DpadLeftState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadLeftState = gamepad2.dpad_left;

            if (gamepad2.dpad_down) {
                telemetry.addData("ngComp", sensors.compensatedTranslate(7 * Math.PI/ 4, false));
            } else if (gamePad2DpadDownState) {
                wheels.softStop(softMotionTime);
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

            if (gamepad1.a) sensors.centerOnZero();

            telemetry.addData("sst", softMotionTime);
            telemetry.addData("rH | iH | head", Utils.toString(Utils.toDegrees(sensors.getRawHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getInitialHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getHeading())));
            telemetry.addData("ng | strafe", Utils.toString(sensors.getNgConstant()) + " | " + Utils.toString(sensors.getStrafeConstant()));
            telemetry.addData("ngRateUp | ngRateDown", sensors.getNgSignChangesPerCycleUpThreshold() + " | " + sensors.getNgSignChangesPerCycleDownThreshold());
            telemetry.addData("blue | red | distance",  Utils.toString(sensors.getBeaconColor()[0]) + " | " + Utils.toString(sensors.getBeaconColor()[1]) + Utils.toString(sensors.getOpticalDistance()));
            telemetry.addData("line readings", Arrays.asList(sensors.getLineReadings()).toString());

            telemetry.update();
        }
    }
}
