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

    private final double[] doorPositions = {0.5, 0.95}; //open and closed respectively

    public Shooter(DcMotor[] disks, Servo door) {
        this.door = door;
        this.disks = disks;

        door.setPosition(doorPositions[1]);

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        disks[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot(double power) {
        //unjam any shots in shooter
        setDiskMotorPowers(-0.5);
        Utils.sleep(200);

        setDiskMotorPowers(Utils.trim(0, 1, power)); //bring motors up to speed

        Utils.sleep(500);

        door.setPosition(doorPositions[0]); //drop door

        Utils.sleep(500);

        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shots through shooter

        Utils.sleep(2500); //wait for shots

        //stop everything
        stop();
        Hardware.getIntake().stopElevator();
        door.setPosition(doorPositions[1]);
        Hardware.getIntake().dropRamp(false);
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

}
