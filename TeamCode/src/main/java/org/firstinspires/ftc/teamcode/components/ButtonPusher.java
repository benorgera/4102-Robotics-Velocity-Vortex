package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 3/31/17.
 */

public class ButtonPusher {

    private Servo left;
    private Servo right;

    public ButtonPusher (Servo left, Servo right) {
        this.left = left;
        this.right = right;

        if (Hardware.isAuton())
            retract();
        else
            semiextend();
    }

    private void retract() { //retract for auton driving
        left.setPosition(1);
        right.setPosition(1);
    }


    private void semiextend() { //semiextend for teleop pushing
        left.setPosition(0.75);
        right.setPosition(0.75);
    }

    public void push(boolean isLeft) {
        (isLeft ? left : right).setPosition(0.3);
        Hardware.sleep(1000);
        (isLeft ? left : right).setPosition(1);
    }


}
