package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 11/24/16.
 */

public class Intake {

    private DcMotor intake;
    private Servo ramp;

    private boolean isRunning = false;

    private double[] rampPositions = {0, 0.5, 1}; //open, holding and closed respectively

    public Intake(DcMotor intake, Servo ramp) {
        this.intake = intake;
        this.ramp = ramp;

        setRampDoor(0);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startIntaking() {
        setRampDoor(0);
        intake.setPower(-.2); //run intake backwards so it doesn't fight the ramp

        isRunning = true;
        new Thread(new DelayedAction(intake, 500, 1)).start(); //concurrently change intake direction after ramp door drops
    }

    public void stopIntaking() {
        setRampDoor(1);
        intake.setPower(0.2); //run intake so it doesn't fight the ramp

        isRunning = false;
        new Thread(new DelayedAction(intake, 500, 0)).start(); //concurrently stop intake after door is brought up
    }

    public void moveRampForShot() {
        setRampDoor(2);
    }

    private void setRampDoor(int positionIndex) {
        ramp.setPosition(rampPositions[positionIndex]);
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
