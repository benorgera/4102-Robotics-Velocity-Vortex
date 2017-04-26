package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.utilities.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 12/1/16.
 */

@TeleOp (name = "Test", group = "4102")
public class Test extends LinearOpMode {

    private final double whiteLineSignalThreshold = 70;

    private boolean gamePad2XState = false;
    private boolean gamepad2YState = false;

    private boolean isUppingNgConstant = false;
    private boolean isDroppingNgConstant = false;

    private boolean isUppingStrafeConstant = false;
    private boolean isDroppingStrafeConstant = false;

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

        Hardware.init(hardwareMap, this, false, true, telemetry);

        wheels = Hardware.getWheels();
        sensors = Hardware.getSensors();

        waitForStart();

        sensors.initImu();

        while (opModeIsActive()) {


            if (gamepad1.back)
                wheels.readyCompensatedTranslate(softMotionTime);

            if (gamepad2.x && ! gamePad2XState)
                softMotionTime += 50;

            gamePad2XState = gamepad1.a;

            if (gamepad2.y && !gamepad2YState && !gamepad1.y)
                    softMotionTime -= 50;


            if (gamepad1.b) wheels.stop();

            //drive if no autonomous driving is occurring
            if (!gamepad1.dpad_down && ! gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad2.dpad_down && ! gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_up) {
                if ((Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0))
                    wheels.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, false);
            }

            if (gamepad1.x) sensors.resetHeading();


            if (gamepad1.dpad_up) {
                sensors.compensatedTranslate(Math.PI / 2);
            } else if (gamePad1DpadUpState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadUpState = gamepad1.dpad_up;

            if (gamepad1.dpad_down) {
                sensors.compensatedTranslate(3 * Math.PI / 2);
            } else if (gamePad1DpadDownState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadDownState = gamepad1.dpad_down;

            if (gamepad1.dpad_right) {
                sensors.compensatedTranslate(0);
            } else if (gamePad1DpadRightState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadRightState = gamepad1.dpad_right;

            if (gamepad1.dpad_left) {
                sensors.compensatedTranslate(Math.PI);
            } else if (gamePad1DpadLeftState) {
                wheels.softStop(softMotionTime);
            }

            gamePad1DpadLeftState = gamepad1.dpad_left;

            if (gamepad2.dpad_right) {
                sensors.compensatedTranslate(Math.PI / 4);
            } else if (gamePad2DpadRightState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadRightState = gamepad2.dpad_right;

            if (gamepad2.dpad_up) {
                sensors.compensatedTranslate(3 * Math.PI / 4);
            } else if (gamePad2DpadUpState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadUpState = gamepad2.dpad_up;

            if (gamepad2.dpad_left) {
                sensors.compensatedTranslate(5 * Math.PI / 4);
            } else if (gamePad2DpadLeftState) {
                wheels.softStop(softMotionTime);
            }

            gamePad2DpadLeftState = gamepad2.dpad_left;

            if (gamepad2.dpad_down) {
                sensors.compensatedTranslate(7 * Math.PI/ 4);
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

            if (gamepad2.b)
                sensors.turn(Math.PI, 0, 0.29);

            if (gamepad1.a) sensors.centerOnZero();

            telemetry.addData("ballSensor", Hardware.getShooter().getAlpha());
            telemetry.addData("touch i | l", sensors.touchSensorPressed(true) + " | " + sensors.touchSensorPressed(false));
            telemetry.addData("range", sensors.getRange());
            telemetry.addData("rH | iH | head", Utils.toString(Utils.toDegrees(sensors.getRawHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getInitialHeading())) + " | " + Utils.toString(Utils.toDegrees(sensors.getHeading())));
            telemetry.addData("ng | strafe", Utils.toString(sensors.getNgConstant()) + " | " + Utils.toString(sensors.getStrafeConstant()));
            telemetry.addData("left | right | ods",  Utils.toString(sensors.getBeaconReadings()[0]) + " | " + Utils.toString(sensors.getBeaconReadings()[1]) + " | " + Utils.toString(sensors.getOpticalDistance()));
            telemetry.addData("line readings", sensors.getLineReadings()[0] + ", " + sensors.getLineReadings()[1]);

            telemetry.update();
        }
    }
}
