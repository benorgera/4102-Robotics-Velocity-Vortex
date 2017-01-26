package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

//        disks[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        disks[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER); //left
//        disks[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //right

        disks[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void shoot(double speed, Telemetry t) {

        //temporary
        for (DcMotor m : disks) {
            m.setTargetPosition(1000);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        setDiskMotorPowers(Utils.trim(0, 1, speed / 10)); //bring motors up to speed

//        Utils.sleep(500);

//        door.setPosition(doorPositions[0]); //drop door
//
//        Utils.sleep(500);

        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shots through shooter

        long stop = System.currentTimeMillis() + 2500;

        while (System.currentTimeMillis() < stop) {
            t.addData("left", disks[0].getCurrentPosition());
            t.addData("right", disks[1].getCurrentPosition());
            t.update();
            Utils.sleep(30);
        }

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
