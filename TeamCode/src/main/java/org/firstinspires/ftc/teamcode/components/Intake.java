package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.DelayedAction;
import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 11/24/16.
 */

public class Intake {

    private DcMotor intake;
    private Servo ramp;
    private Servo[] flaps;
    private CRServo[] spinners;

    private boolean isRampDown;

    private boolean hasFourthBall = false;
    private boolean isRunning = false;

    private final double[] rampPositions = {0.03, 0.1325, 0.235, 392}; //down, holding 4, holding 3, and closed respectively

    public Intake(DcMotor intake, Servo ramp, Servo[] flaps, CRServo[] spinners, ColorSensor ballSensor) {
        this.intake = intake;
        this.ramp = ramp;
        this.flaps = flaps;
        this.spinners = spinners;

        ramp.setPosition(rampPositions[(isRampDown = Hardware.isAuton()) ? 2 : 0]);
        setFlaps(false);

        if (Hardware.isAuton()) {
            runSpinners(); //run spinners to free them
            Hardware.sleep(1000);
            stopSpinners();
        }

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
        ramp.setPosition(rampPositions[hasFourthBall ? 1 : 2]);
        intake.setPower(1); //run intake so it doesn't fight the ramp

        isRunning = isRampDown = false;
        new Thread(new DelayedAction(intake, 500, 0)).start(); //concurrently stop intake after door is brought up

        stopSpinners();
    }

    public void dropRamp(double intakePower) {
        ramp.setPosition(rampPositions[0]);
        isRampDown = true;
        intake.setPower(-1); //run intake backwards so it doesn't fight the ramp

        new Thread(new DelayedAction(intake, 300, intakePower)).start(); //concurrently change intake direction after ramp door drops
    }

    public void stop() {
        stopSpinners();
        intake.setPower(0);
    }

    public boolean isRunning() {
        return isRunning;
    }

    private void runSpinners() {
        spinners[0].setPower(1);
        spinners[1].setPower(-1);
    }

    private void stopSpinners() {
        for (CRServo s : spinners)
            s.setPower(0);
    }

    public void setFlaps(boolean areOut) {
        flaps[0].setPosition(areOut ? 0 : 0.25);
        flaps[1].setPosition(areOut ? 1 : 0.75);
    }

    public boolean toggleHasFourthBall() {
        hasFourthBall = !hasFourthBall;

        return hasFourthBall;
    }

    public boolean hasFourthBall() {
        return hasFourthBall;
    }

    public void runElevator(double power) {
        intake.setPower(power);
    }

    public void takeShot() {
        isRampDown = false;
        ramp.setPosition(rampPositions[3]); //move ramp out of way of intake
        intake.setPower(1); //feed shot through the shooter
    }


    public void stopShooting() {
        stop();
        dropRamp(0);
        hasFourthBall = false;
    }


}
