package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 3/31/17.
 */

public class ButtonPusher {

    private Servo[] pushers; //left then right

    public ButtonPusher (Servo[] pushers) {
        this.pushers = pushers;

        if (Hardware.isAuton())
            retract();
        else
            semiextend();
    }

    private void retract() { //retract for auton driving
        setPositions(1);
    }


    private void semiextend() { //semiextend for teleop pushing
        setPositions(0.75);
    }

    private void setPositions(double position) {
        for (Servo s : pushers) s.setPosition(position);
    }


    public void push(boolean isLeft) {
        pushers[isLeft ? 0 : 1].setPosition(0.17);
        Hardware.sleep(1000);
        retract();
    }

    public void pushBoth() {
        setPositions(0.3);
        Hardware.sleep(1000);
        retract();
    }



}
