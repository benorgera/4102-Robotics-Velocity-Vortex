package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by benorgera on 11/24/16.
 */

public class Intake {

    private DcMotor intake;

    public Intake(DcMotor intake) {
        this.intake = intake;

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        intake.setPower(1);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void holdIMU() { //slowly run the intake, so that the conveyor paddles press up against the gyro plate and hold it steady
        intake.setPower(0.2);
    }

    public void releaseIMU() {
        intake.setPower(-0.2);
    }

}
