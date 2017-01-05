package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by benorgera on 11/24/16.
 */

public class Shooter {

    private DcMotor[] disks;
    private Servo door;

    private double[] doorPositions = {0, 1}; //open closed respectively

    public Shooter(DcMotor[] disks, Servo door) {
        this.door = door;
        this.disks = disks;

        door.setPosition(doorPositions[1]);

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        disks[0].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot(double power) {

        setDiskMotorPowers(Utils.trim(0, 1, power)); //bring motors up to speed

        Utils.sleep(500);

        door.setPosition(doorPositions[0]); //drop door

        Utils.sleep(500);

        Hardware.getIntake().startElevator(); //feed shots through shooter

        Utils.sleep(100); //wait for first shot

        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake

        Utils.sleep(300); //wait for other shots

        //stop everything
        stop();
        Hardware.getIntake().stopElevator();
        door.setPosition(doorPositions[1]);
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

}
