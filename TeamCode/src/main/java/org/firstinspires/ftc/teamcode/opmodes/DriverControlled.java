package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 10/24/16.
 */
@TeleOp(name = "Shmoney Yerd L17", group = "4102")
public class DriverControlled extends LinearOpMode {

    //modes
    private boolean isChannelMode = true; //left bumper to enable channel mode, right bumper to disable channel mode

    //components
    private Lift lift;
    private Shooter shooter;
    private Wheels wheels;
    private Intake intake;

    private int shotPower = 6;
    
    private boolean wasTogglingDirection = false;

    private boolean intakeIsFront = false;

    private boolean isHoldingLift = false;
    private boolean wasTogglingHoldingLift = false;

    private boolean wasUppingShotPower = false;
    private boolean wasDowningShotPower = false;

    private boolean wasShooting = false;

    private boolean wasTogglingIntake = false;

    private boolean hasDroppedFork = false;

    private long startTime;

    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, false);

        lift = Hardware.getLift();
        wheels = Hardware.getWheels();
        intake = Hardware.getIntake();
        shooter = Hardware.getShooter();

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
        telemetry.addData("WHEELS", wheels.drive((intakeIsFront ? 1 : -1) * gamepad1.left_stick_x, (intakeIsFront ? -1 : 1) * gamepad1.left_stick_y, gamepad1.right_stick_x, isChannelMode = gamepad1.left_bumper || (!gamepad1.right_bumper && isChannelMode)));

        if (gamepad1.a && !wasTogglingDirection)
            intakeIsFront = !intakeIsFront;

        wasTogglingDirection = gamepad1.a;


        //--------------------------SHOOTER-------------------------------

        if (gamepad2.dpad_right && !wasUppingShotPower)
            shotPower = (int) Utils.trim(0, 10, shotPower + 1);

        wasUppingShotPower = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !wasDowningShotPower)
            shotPower = (int) Utils.trim(0, 10, shotPower - 1);

        wasDowningShotPower = gamepad2.dpad_left;

        if (gamepad2.b && !intake.isRunning() && !wasShooting)
            shooter.shoot(shotPower, telemetry);

        wasShooting = gamepad2.b;




        //--------------------------Intake-------------------------------

        if (gamepad2.a && !wasTogglingIntake)
            if (intake.isRunning())
                intake.stopIntaking();
            else
                intake.startIntaking(!gamepad2.left_bumper);

        wasTogglingIntake = gamepad2.a;


        //--------------------------LIFT-------------------------------

        //drop fork if it hasn't been dropped before
        if (!hasDroppedFork && (hasDroppedFork = gamepad2.x))
            lift.dropFork();

        if (gamepad2.y && !wasTogglingHoldingLift && hasDroppedFork)
            isHoldingLift = !isHoldingLift;

        wasTogglingHoldingLift = gamepad2.y;

        if (gamepad2.dpad_up && hasDroppedFork) //raise the lift if we've dropped the fork
            lift.raise();
        else if (gamepad2.dpad_down && hasDroppedFork) //lower the lift if we've dropped the fork
            lift.lower();
        else if (isHoldingLift)
            lift.hold();
        else
            lift.stop();

        telemetry.addData("FRONT", intakeIsFront ? "INTAKE" : "SHOOTER");
        telemetry.addData("SHOT", shotPower);
        if (isHoldingLift) telemetry.addData("LIFT", "HOLDING");
        telemetry.addData("TIME", getTimeString());
        telemetry.update();
    }

    private String getTimeString() { //get remaining match time as a string
        int deltaSeconds = 120 - (int) (System.currentTimeMillis() - startTime) / 1000,
                deltaMin = deltaSeconds / 60,
                seconds = deltaSeconds % 60;

        return "" + deltaMin + ":" + (("" + seconds).length() == 1 ? "0" + seconds : seconds) + (seconds <= 30 && deltaMin == 0 ? "  END GAME" : "");
    }
}
