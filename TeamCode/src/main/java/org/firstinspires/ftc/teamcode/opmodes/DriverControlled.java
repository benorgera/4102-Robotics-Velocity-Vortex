package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by benorgera on 10/24/16.
 */
@TeleOp(name="Shmoney Yerd L17", group="4102")
public class DriverControlled extends LinearOpMode {

    //modes
    private boolean isChannelMode = true; //left bumper to enable channel mode, right bumper to disable channel mode

    //components
    private Wheels wheels;

    private String log = "";
    private long startTime;

    @Override
    public void runOpMode() {
        Hardware.setMap(hardwareMap);
        wheels = Hardware.getWheels();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the match to start

        startTime = System.currentTimeMillis(); //store the start time, so we can print remaining match time

        while (opModeIsActive()) run(); //control to robot using gamepad input while the opMode is active

        wheels.stop(); //stop the robot once the opMode is disabled
        telemetry.addData("Saving Log", Utils.writeStringToFile(hardwareMap.appContext, log, new SimpleDateFormat("MM/dd/yy HH:mm:ss 'log'").format(new Date())) ? "Successful" : "Failed"); //save the robot actions log file
        telemetry.update();
    }

    private void run() { //control to robot using gamepad input

        //all components return a string with telemetry data when passed input
        telemetry.addData("WHEELS", addToLog(wheels.drive(-1 * gamepad1.left_stick_x, -1 * gamepad1.left_stick_y, gamepad1.right_stick_x, isChannelMode = gamepad1.left_bumper || (!gamepad1.right_bumper && isChannelMode))));
        telemetry.addData("TIME", addToLog(getTimeString()));

        telemetry.update();
    }

    private String addToLog(String s) { //add string to log of robot actions
        log += (s + "\n\n");
        return s;
    }

    private String getTimeString() { //get remaining match time as a string
        int deltaSeconds = 120 - (int) (System.currentTimeMillis() - startTime) / 1000;
        String seconds = "" + deltaSeconds % 60;

        return "" + deltaSeconds / 60 + ":" + (seconds.length() == 1 ? "0" + seconds : seconds);
    }
}
