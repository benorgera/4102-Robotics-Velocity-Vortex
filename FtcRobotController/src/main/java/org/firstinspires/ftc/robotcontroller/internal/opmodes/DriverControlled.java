package org.firstinspires.ftc.robotcontroller.internal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcontroller.internal.components.Wheels;

/**
 * Created by benorgera on 10/24/16.
 */
@TeleOp(name="Shmoney Yerd L17", group="4102")
public class DriverControlled extends LinearOpMode {

    //modes
    private boolean isChannelMode = true; //left bumper to enable channel mode, right bumper to disable channel mode

    //components
    private Wheels wheels;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) run(); //control to robot using gamepad input while the opMode is active

        wheels.stop(); //stop the robot once the opMode is disabled
    }

    private void run() { //control to robot using gamepad input

        //all components return a string with telemetry data when passed input
        telemetry.addData("WHEELS", wheels.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, isChannelMode = gamepad1.left_bumper || (!gamepad1.right_bumper && isChannelMode)));


        telemetry.addData("Hello", "Van");
        telemetry.update();
    }

    private void initializeHardware() { //initialize hardware according to hardware map
        wheels = new Wheels(
            hardwareMap.dcMotor.get("front-left-wheel"),
            hardwareMap.dcMotor.get("front-right-wheel"),
            hardwareMap.dcMotor.get("back-left-wheel"),
            hardwareMap.dcMotor.get("back-right-wheel")
        );
    }

}
