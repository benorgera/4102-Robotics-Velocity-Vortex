package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 11/24/16.
 */

public class Shooter {

    private DcMotor[] disks;
    private Servo door;

    private final double[] doorPositions = {0.333, 1}; //open and closed respectively

    public Shooter(DcMotor[] disks, Servo door) {
        this.door = door;
        this.disks = disks;

        door.setPosition(doorPositions[1]);

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        disks[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void shoot(int speed) {

        setDiskMotorPowers(Utils.trim(0, 7 / 9, speed * (7 / 90))); //bring motors up to speed

        Utils.sleep(500);

        door.setPosition(doorPositions[0]); //drop door

        Utils.sleep(500);

        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shots through shooter

        Utils.sleep(2500); //wait for shots

        //stop everything
        stop();
        Hardware.getIntake().stopElevator();

        //reset ramp for next intaking
        door.setPosition(doorPositions[1]);
        Hardware.getIntake().dropRamp(0);
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

}
