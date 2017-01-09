package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 1/4/17.
 */

public class Lift {

    private DcMotor lift;
    private Servo latch;

    private final double[] latchPositions = {1, 0}; //latched and unlatched respectively

    public Lift(DcMotor lift, Servo latch) {
        this.lift = lift;
        this.latch = latch;

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        latch.setPosition(latchPositions[0]);
    }

    public void dropFork() {
        latch.setPosition(latchPositions[1]);

        new Thread(new DelayedAction(latch, 500, latchPositions[0])).start();
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


}
