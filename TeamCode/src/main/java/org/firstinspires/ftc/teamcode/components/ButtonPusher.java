package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.DelayedAction;

/**
 * Created by benorgera on 3/31/17.
 */

public class ButtonPusher {

    private Servo left;
    private Servo right;

    public ButtonPusher (boolean isAuton, Servo left, Servo right) {
        this.left = left;
        this.right = right;

        if (isAuton)
            retract();
        else
            semiextend();
    }

    private void retract() { //retract for auton driving
        left.setPosition(0);
        right.setPosition(1);
    }


    private void semiextend() { //semiextend for teleop pushing
        left.setPosition(0.25);
        right.setPosition(0.75);
    }

    public void push(boolean isLeft) {
        if (isLeft) {
            left.setPosition(0.5);
            (new Thread(new DelayedAction(left, 400, 0))).start();
        } else {
            right.setPosition(0.5);
            (new Thread(new DelayedAction(right, 400, 1))).start();
        }
    }


}