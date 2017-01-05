package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
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
    private Lift lift;
    private Shooter shooter;
    private Wheels wheels;
    private Intake intake;
    private Sensors sensors;

    double shotPower = 0;

    private boolean wasUppingShotPower = false;
    private boolean wasDowningShotPower = false;

    private boolean wasShooting = false;

    private boolean wasTogglingIntake = false;

    private boolean hasDroppedFork = false;

    private long startTime;

    @Override
    public void runOpMode() {
        Hardware.setMap(hardwareMap);

        lift = Hardware.getLift();
        wheels = Hardware.getWheels();
        intake = Hardware.getIntake();
        shooter = Hardware.getShooter();
        sensors = Hardware.getSensors();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the match to start

        startTime = System.currentTimeMillis(); //store the start time, so we can print remaining match time

        while (opModeIsActive()) run(); //control to robot using gamepad input while the opMode is active

        Hardware.freezeAllMotorFunctions(); //stop all motors in case any are running

        telemetry.addData("4102", "We kicked up");
        telemetry.update();
    }

    private void run() { //control to robot using gamepad input

        //all components return a string with telemetry data when passed input
        telemetry.addData("WHEELS", wheels.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, isChannelMode = gamepad1.left_bumper || (!gamepad1.right_bumper && isChannelMode)));
        telemetry.addData("SHOT", shotPower);

        //--------------------------SHOOTER-------------------------------

        if (gamepad2.dpad_right && !wasUppingShotPower)
            shotPower = Utils.trim(0, 1, shotPower + 0.1);

        wasUppingShotPower = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !wasDowningShotPower)
            shotPower = Utils.trim(0, 1, shotPower - 0.1);

        wasDowningShotPower = gamepad2.dpad_left;

        if (gamepad2.b && !intake.isRunning() && !wasShooting)
            shooter.shoot(shotPower);

        wasShooting = gamepad2.b;




        //--------------------------Intake-------------------------------

        if (gamepad2.a && !wasTogglingIntake)
            if (intake.isRunning())
                intake.stopIntaking();
            else
                intake.startIntaking();

        wasTogglingIntake = gamepad2.a;





        //--------------------------LIFT-------------------------------

        //drop fork if it hasn't been dropped before
        if (!hasDroppedFork && (hasDroppedFork = gamepad2.x))
            lift.dropFork();

        if (gamepad2.dpad_up && hasDroppedFork) //raise the lift if we've dropped the fork
            lift.raise();
        else if (gamepad2.dpad_down && hasDroppedFork) //lower the lift if we've dropped the fork
            lift.lower();
        else
            lift.stop();



        telemetry.addData("TIME", getTimeString());
        telemetry.update();
    }

    private String getTimeString() { //get remaining match time as a string
        int deltaSeconds = 120 - (int) (System.currentTimeMillis() - startTime) / 1000;
        String seconds = "" + deltaSeconds % 60;

        return "" + deltaSeconds / 60 + ":" + (seconds.length() == 1 ? "0" + seconds : seconds);
    }
}
