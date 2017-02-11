package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 1/5/17.
 */

public class DelayedAction implements Runnable {

    private Object o;
    private long delay;
    private double setting;

    public DelayedAction(Object o, long delay, double setting) {
        this.o = o;
        this.delay = delay;
        this.setting = setting;
    }

    @Override
    public void run() {
        Hardware.sleep(delay);

        if (o instanceof DcMotor)
            ((DcMotor) o).setPower(setting);
        else
            ((Servo) o).setPosition(setting);
    }


}
