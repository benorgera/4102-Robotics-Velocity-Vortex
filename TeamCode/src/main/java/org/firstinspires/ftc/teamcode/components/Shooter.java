package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 11/24/16.
 */

public class Shooter {

    private DcMotor[] disks;
    private Servo door;
    private ColorSensor ballSensor;

    private final double[] doorPositions = {0.333, 1}; //open and closed respectively

    private final double alphaThreshold = 100;

    public Shooter(DcMotor[] disks, Servo door, ColorSensor ballSensor) {
        this.door = door;
        this.disks = disks;
        this.ballSensor = ballSensor;

        door.setPosition(doorPositions[1]);

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        disks[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void shoot(double speed) {
        ballSensor.enableLed(true); //turn on LED so we can sense the ball
        Hardware.getWheels().stop(); //stop the robot in case its moving
        door.setPosition(doorPositions[0]); //open shooter

        PID controller = new PID(disks, 0.55, 0.55, 0.1, 0.25); //initialize speed controller

        Thread runnable = new Thread(controller);
        runnable.start(); //bring motors up to speed by starting the PID

        while (takeShot(controller)); //while there's another ball remaining, shoot

        runnable.interrupt(); //stop the PID

        //reset stuff
        ballSensor.enableLed(false);
        Hardware.getIntake().stopElevator();
        stop();
        door.setPosition(doorPositions[1]); //reset ramp for next intaking
        Hardware.getIntake().dropRamp(0);
    }

    //take shot while regulating speed, stopping elevator after first ball passes through
    //return true if another ball remains, and therefore another shot should be taken
    private boolean takeShot(PID controller) {
        while (!controller.ready()) //wait while the motors aren't up to speed
            Hardware.sleep(10);

        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shot through the shooter

        //wait for the next ball to be in position
        //if this command times out, false is returned and no balls remain so no more shots should be taken
        boolean takingAnotherShot = waitForNextBall();

        Hardware.getIntake().stopElevator(); //stop elevator

        //tell the controller the motors are no longer at the right speed, so compensation is once again applied
        controller.reset();

        return takingAnotherShot; //return true if another shot is to be taken
    }

    //wait for the next ball to be sensed, and return true unless another ball is never found
    private boolean waitForNextBall() {
        while (hasBall()) //wait for the ball currently being shot to pass through
            Hardware.sleep(10);

        long lastHadBall = System.currentTimeMillis();

        boolean foundAnotherBall = true;

        //continue running the elevator (waiting) until you see a new ball, or you time out
        //waiting will only cease after at least 50ms have passed since last sensing a ball
        while (!hasBall() || System.currentTimeMillis() - lastHadBall < 50 && (foundAnotherBall = System.currentTimeMillis() - lastHadBall < 200))
            Hardware.sleep(10);

        return foundAnotherBall; //return true unless no additional balls were found
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    private boolean hasBall() {
        return ballSensor.alpha() > alphaThreshold;
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

}
