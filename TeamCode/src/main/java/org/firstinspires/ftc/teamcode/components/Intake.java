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

    private boolean isRunning = false;

    private final double[] rampPositions = {0.4, 0.7, 1}; //open, holding and closed respectively

    public Intake(DcMotor intake, Servo ramp, boolean isAuton) {
        this.intake = intake;
        this.ramp = ramp;

        ramp.setPosition(rampPositions[isAuton ? 1 : 0]);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startIntaking() {
        isRunning = true;
        ramp.setPosition(rampPositions[0]);
        intake.setPower(-1); //run intake backwards so it doesn't fight the ramp

        new Thread(new DelayedAction(intake, 500, 1)).start(); //concurrently change intake direction after ramp door drops
    }

    public void stopIntaking() {
        ramp.setPosition(rampPositions[1]);
        intake.setPower(1); //run intake so it doesn't fight the ramp

        isRunning = false;
        new Thread(new DelayedAction(intake, 500, 0)).start(); //concurrently stop intake after door is brought up
    }

    public void moveRampForShot() {
        ramp.setPosition(rampPositions[2]);
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
