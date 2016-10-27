package org.firstinspires.ftc.robotcontroller.internal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.Utils;
import org.firstinspires.ftc.robotcontroller.internal.components.Wheels;
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
        initializeHardware();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the driver station to

        startTime = System.currentTimeMillis(); //store the start time, so we can print remaining match time

        while (opModeIsActive()) run(); //control to robot using gamepad input while the opMode is active

        wheels.stop(); //stop the robot once the opMode is disabled
        telemetry.addData("Saving Log", Utils.writeToStringToFile(hardwareMap.appContext, log, new SimpleDateFormat("MM/dd/yy HH:mm:ss 'log'").format(new Date())) ? "Successful" : "Failed"); //save the robot actions log file
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


    private void initializeHardware() { //initialize hardware according to hardware map
        wheels = new Wheels(
            hardwareMap.dcMotor.get("front-left-wheel"),
            hardwareMap.dcMotor.get("front-right-wheel"),
            hardwareMap.dcMotor.get("back-left-wheel"),
            hardwareMap.dcMotor.get("back-right-wheel")
        );
    }

    private String getTimeString() { //get remaining match time as a string
        int deltaSeconds = 120 - (int) (System.currentTimeMillis() - startTime) / 1000;
        String seconds = "" + deltaSeconds % 60;

        return "" + deltaSeconds / 60 + ":" + (seconds.length() == 1 ? "0" + seconds : seconds);
    }
}
