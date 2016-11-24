package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by benorgera on 11/24/16.
 */

public class Shooter {

    private DcMotor[] disks;

    public Shooter(DcMotor[] disks) {
        this.disks = disks;

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void run(double initialVelocity) { //run shooter with desired initial shot velocity
        setDiskMotorPowers(Utils.trim(0, 1, initialVelocityToMotorPower(initialVelocity)));
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    private double initialVelocityToMotorPower(double initialVelocity) { //convert desired initial velocity for ball into motor speed on shooter
        return initialVelocity;
    }

}
