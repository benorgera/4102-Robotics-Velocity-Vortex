package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 11/24/16.
 */

public class Intake {

    private DcMotor intake;
    private Servo ramp;

    private boolean isRampDown;

    private boolean isRunning = false;

    private final double[] rampPositions = {0.45, 0.8, 1}; //down, holding and closed respectively

    public Intake(DcMotor intake, Servo ramp, boolean isAuton) {
        this.intake = intake;
        this.ramp = ramp;

        ramp.setPosition(rampPositions[(isRampDown = !isAuton) ? 0 : 1]);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startIntaking() {
        isRunning = true;

        if (isRampDown)
            intake.setPower(1);
        else
            dropRamp(true);
    }

    public void stopIntaking() {
        ramp.setPosition(rampPositions[1]);
        intake.setPower(1); //run intake so it doesn't fight the ramp

        isRunning = isRampDown = false;
        new Thread(new DelayedAction(intake, 500, 0)).start(); //concurrently stop intake after door is brought up
    }

    public void moveRampForShot() {
        isRampDown = false;
        ramp.setPosition(rampPositions[2]);
    }

    public void dropRamp(boolean shouldStartIntake) {
        ramp.setPosition(rampPositions[0]);
        isRampDown = true;
        intake.setPower(-1); //run intake backwards so it doesn't fight the ramp

        new Thread(new DelayedAction(intake, 500, shouldStartIntake ? 1 : 0)).start(); //concurrently change intake direction after ramp door drops
    }

    public void startElevator() {
        intake.setPower(1);
    }

    public void stopElevator() {
        intake.setPower(0);
    }

    public boolean isRunning() {
        return isRunning;
    }
}
