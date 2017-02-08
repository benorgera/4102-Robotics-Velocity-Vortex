package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.DelayedAction;

/**
 * Created by benorgera on 1/4/17.
 */

public class Lift {

    private DcMotor lift;
    private Servo forkLatch;

    private Servo sliderLock;

    private final double[] lockPositions = {0.6, 0.9}; //locked and unlocked respectively

    private final double[] latchPositions = {0.93, 0}; //latched and unlatched respectively

    public Lift(DcMotor lift, Servo forkLatch, Servo sliderLock) {
        this.lift = lift;
        this.forkLatch = forkLatch;

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        forkLatch.setPosition(latchPositions[0]);
        sliderLock.setPosition(lockPositions[0]);

    }

    public void dropFork() {
        forkLatch.setPosition(latchPositions[1]);

        new Thread(new DelayedAction(forkLatch, 500, latchPositions[0])).start();
    }

    public void unlock() {
        sliderLock.setPosition(lockPositions[1]);
    }

    public void raise() {
        lift.setPower(1);
    }

    public void stop() {
        lift.setPower(0);
    }

    public void lower() {
        lift.setPower(-0.5);
    }

    public void hold() {
        lift.setPower(0.35);
    }
}
