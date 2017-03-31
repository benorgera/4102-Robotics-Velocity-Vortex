package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.DelayedAction;

/**
 * Created by benorgera on 11/24/16.
 */

public class Intake {

    private DcMotor intake;
    private Servo ramp;
    private Servo[] flaps;
    private Servo[] spinners;

    private boolean isRampDown;

    private boolean isRunning = false;

    private final double[] rampPositions = {0.49, 0.77, 0.9}; //down, holding and closed respectively

    public Intake(DcMotor intake, Servo ramp, boolean isAuton, Servo[] flaps, Servo[] spinners) {
        this.intake = intake;
        this.ramp = ramp;
        this.flaps = flaps;
        this.spinners = spinners;

        ramp.setPosition(rampPositions[(isRampDown = !isAuton) ? 0 : 1]);
        setFlaps(false);
        runSpinners(); //run spinners to free them

        for (Servo s : spinners) //stop spinners in one second
            (new Thread(new DelayedAction(s, 1000, 0.5))).start();

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void startIntaking(boolean shouldRunForwards) {
        isRunning = true;

        if (isRampDown)
            intake.setPower(shouldRunForwards ? 1 : -1);
        else
            dropRamp(shouldRunForwards ? 1 : -1);

        runSpinners();
    }

    public void stopIntaking() {
        ramp.setPosition(rampPositions[1]);
        intake.setPower(1); //run intake so it doesn't fight the ramp

        isRunning = isRampDown = false;
        new Thread(new DelayedAction(intake, 500, 0)).start(); //concurrently stop intake after door is brought up

        stopSpinners();
    }

    public void moveRampForShot() {
        isRampDown = false;
        ramp.setPosition(rampPositions[2]);
    }

    public void dropRamp(double intakePower) {
        ramp.setPosition(rampPositions[0]);
        isRampDown = true;
        intake.setPower(-1); //run intake backwards so it doesn't fight the ramp

        new Thread(new DelayedAction(intake, 500, intakePower)).start(); //concurrently change intake direction after ramp door drops
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

    public void runElevator(double power) {
        intake.setPower(power);
    }

    private void runSpinners() {
        spinners[0].setPosition(1);
        spinners[1].setPosition(0);
    }

    private void stopSpinners() {
        for (Servo s : spinners)
            s.setPosition(0.5);
    }

    public void setFlaps(boolean areOut) {
        flaps[0].setPosition(areOut ? 0 : 0.25);
        flaps[1].setPosition(areOut ? 1 : 0.75);
    }

}
