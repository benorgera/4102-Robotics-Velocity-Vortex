package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.utilities.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 10/24/16.
 */
@TeleOp(name = "Shmoney Yerd L17", group = "4102")
public class DriverControlled extends LinearOpMode {

    //modes
    private boolean isSlowMode = false;
    private boolean intakeIsFront = true;

    //components
    private Lift lift;
    private Shooter shooter;
    private Wheels wheels;
    private Intake intake;

    private double shotSleep = 200;

    private boolean wasTogglingHasFourthBall = false;
    private boolean wasPreppingShot = false;
    private boolean wasTogglingDirection = false;
    private boolean wasTogglingSlowMode = false;
    private boolean wasUppingShotSleep = false;
    private boolean wasDowningShotSleep = false;
    private boolean wasShooting = false;
    private boolean wasTogglingIntake = false;
    private boolean wasDroppingFork = false;

    private boolean hasDroppedFork = false;

    private boolean isPreppingShot = false;

    private long startTime;

    private final double slowModeConstant = 0.5;

    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap, this, false, false, telemetry); //initializes hardware

        Hardware.getButtonPusher();

        lift = Hardware.getLift();
        wheels = Hardware.getWheels();
        intake = Hardware.getIntake();
        shooter = Hardware.getShooter();

        telemetry.addData("4102", "Let's kick up");
        telemetry.update();

        waitForStart(); //wait for the match to start (start button to be pressed)

        startTime = System.currentTimeMillis(); //store the start time, so we can print remaining match time

        while (Hardware.active()) run(); //control to robot using gamepad input while the opMode is active

        Hardware.freezeAllMotorFunctions(); //stop all motors in case any are running

        telemetry.addData("4102", "We kicked up");
        telemetry.update();
    }

    private void run() { //control to robot using gamepad input


        //--------------------------DRIVING-------------------------------

        drive(); //send joystick commands to wheels

        if (gamepad1.a && !wasTogglingDirection)
            intakeIsFront = !intakeIsFront; //a on the first gamepad toggles which side of the robot is the front so we can adjust for shooting vs intaking

        wasTogglingDirection = gamepad1.a;

        if (gamepad1.b && !wasTogglingSlowMode)
            isSlowMode =! isSlowMode; //b on the first gamepad toggles slow mode, which allows us to be more precise in our driving

        wasTogglingSlowMode = gamepad1.b;


        //--------------------------SHOOTER-------------------------------

        if (gamepad2.dpad_right && !wasUppingShotSleep) //the right dpad button on the second controller allows us to increase the power of our shot
            shotSleep = Utils.trim(0, 1000, shotSleep + 100);

        wasUppingShotSleep = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !wasDowningShotSleep) //the left dpad button on the second controller allows us to decrease the power of our shot
            shotSleep = Utils.trim(0, 1000, shotSleep - 100);

        wasDowningShotSleep = gamepad2.dpad_left;

        if (gamepad2.y && !wasPreppingShot && !isPreppingShot && !intake.isRunning()) { //y on the second controller starts to get our shot motors up to speed
            isPreppingShot = true;
            shooter.prepShot(6);
        } else if (gamepad2.y && !wasPreppingShot && !intake.isRunning()){ //if y is pressed again, the shooter resets without shooting
            shooter.stop();
            shooter.close();
            isPreppingShot = false;
        }

        wasPreppingShot = gamepad2.y;

        if (gamepad2.b && !intake.isRunning() && isPreppingShot && !wasShooting) { //we take a shot if b on the second controller is pressed and the intake isn't running, and the shooter has been prepped
            shooter.shoot((long) shotSleep);
            isPreppingShot = false;
        }

        wasShooting = gamepad2.b;


        //--------------------------Intake-------------------------------

        intake.setFlaps(gamepad2.right_bumper);

        if (gamepad2.a && !wasTogglingIntake && !isPreppingShot) //gamepad 2 a toggles the intake on and off
            if (intake.isRunning())
                intake.stopIntaking();
            else
                intake.startIntaking(!gamepad2.left_bumper);

        wasTogglingIntake = gamepad2.a;

        if (gamepad2.start && !wasTogglingHasFourthBall && intake.toggleHasFourthBall() && intake.isRunning()) //toggle has fourth ball, if there is one, and the intake's running, stop the intake and hold four balls
            intake.stopIntaking();

        wasTogglingHasFourthBall = gamepad2.start;



        //--------------------------LIFT-------------------------------

        if (gamepad2.x && !wasDroppingFork) { //pressing x on the second gamepad releases the servo holding the fork
            hasDroppedFork = true;
            lift.dropFork();
        }

        wasDroppingFork = gamepad2.x;

        if (gamepad2.dpad_up && hasDroppedFork) //raise the lift if we've dropped the fork
            lift.raise();
        else if (gamepad2.dpad_down && hasDroppedFork) //lower the lift if we've dropped the fork
            lift.lower();
        else
            lift.stop();


        //--------------------------DATA-------------------------------

        telemetry.addData("SHOT SLEEP", "" + shotSleep);
        telemetry.addData("MODE", intakeIsFront ? "INTAKE" : "SHOOT");
        if (isSlowMode) telemetry.addData("SLOW MODE", "TRUE");
        if (intake.hasFourthBall()) telemetry.addData("FOURTH BALL", "TRUE");
        telemetry.addData("TIME", getTimeString());
        telemetry.update();
    }

    private String getTimeString() { //get remaining match time as a string
        int deltaSeconds = 120 - (int) (System.currentTimeMillis() - startTime) / 1000,
                deltaMin = deltaSeconds / 60,
                seconds = deltaSeconds % 60;

        return "" + deltaMin + ":" + (("" + seconds).length() == 1 ? "0" + seconds : seconds) + (seconds <= 30 && deltaMin == 0 ? "  END GAME" : "");
    }

    public void drive() {
        //drives according to the left and right joysticks– left joystick drives, and right joystick rotates. if in slowmode, the robot can drive at lower speeds, and if not in front intake mode, the joysticks have opposite effects
        wheels.drive((isSlowMode ? slowModeConstant : 1) * (intakeIsFront ? 1 : -1) * gamepad1.left_stick_x, (isSlowMode ? slowModeConstant : 1) * (intakeIsFront ? -1 : 1) * gamepad1.left_stick_y, (isSlowMode ? slowModeConstant : 1) * gamepad1.right_stick_x, !isSlowMode); //dr
    }
}
