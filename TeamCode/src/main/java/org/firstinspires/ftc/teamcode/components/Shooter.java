package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private final double[] doorPositions = {0.3, 1}; //open and closed respectively

    private final double alphaThreshold = 55;

    public Shooter(DcMotor[] disks, Servo door, ColorSensor ballSensor) {
        this.door = door;
        this.disks = disks;
        this.ballSensor = ballSensor;

        close();

        ballSensor.enableLed(true); //turn on LED so we can sense the ball

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        disks[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void prepShot(double speed) {
        open(); //open shooter
        setDiskMotorPowers(speed / 10); //bring motors up to speed by starting the PID
    }

    public void shoot(double speed, boolean isAuton) {
        setDiskMotorPowers(speed / 10); //adjust speed (maybe it was prepped lower)
        Hardware.getWheels().stop(); //stop the robot in case its moving

        int count = 0;

        //while there's another ball remaining, shoot
        while (takeShot((count == 1 && isAuton) || count == 2) && Hardware.active()) count++;

        //reset stuff
        Hardware.getIntake().stopElevator();
        stop(); //stop the PID
        close();
        Hardware.getIntake().dropRamp(0); //reset ramp for next intaking
    }

    //take shot while regulating speed, stopping elevator after first ball passes through
    //return true if another ball remains, and therefore another shot should be taken
    private boolean takeShot(boolean isLastShot) {
        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shot through the shooter



        //wait for the next ball to be in position
        //if this command times out, false is returned and no balls remain so no more shots should be taken
        boolean takingAnotherShot = waitForNextBall(isLastShot);

        if (takingAnotherShot) { //about to shoot again
            //supply negative power to stop the elevator quickly
            Hardware.getIntake().runElevator(-0.3);
            Hardware.sleep(20);

            Hardware.getIntake().stopElevator(); //stop elevator
            Hardware.sleep(700); //wait for PID to regain desired velocity
        } else { //no shots remain, stop and return
            Hardware.getIntake().stopElevator();
        }

        return takingAnotherShot; //return true if another shot is to be taken
    }

    //wait for the next ball to be sensed, and return true unless another ball is never found
    private boolean waitForNextBall(boolean isLastShot) {

        long started = System.currentTimeMillis();

        while (hasBall() && Hardware.active()); //wait for the ball currently being shot to pass through

        long lastHadBall = System.currentTimeMillis();

        Hardware.print("Had ball for " + (lastHadBall - started) + " ms");

        //we know this was our last shot because it was shot 2 in auton or 3 in teleop
        if (isLastShot) {
            Hardware.sleep(350); //wait for the last shot to pass
            return false; //don't shoot again
        }

        long waitForBall = 1000; //how long the shooter waits to see another ball before assuming none remain

        //continue running the elevator (waiting) until you see a new ball, or you time out
        //waiting will only cease after at least 50ms have passed since last sensing a ball
        while (Hardware.active() && !hasBall() && (System.currentTimeMillis() - lastHadBall) < waitForBall || System.currentTimeMillis() - lastHadBall < 150);

        Hardware.print("Waited " + (System.currentTimeMillis() - lastHadBall) + "ms for next ball");
        return System.currentTimeMillis() - lastHadBall < waitForBall; //return true unless no additional balls were found
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    //returns true if a ball is sensed in the elevator, ready to be shot
    private boolean hasBall() {
        long stop = System.currentTimeMillis() + 75; //take readings for 75 ms

        double total = 0;
        int count = 0;

        //average the color sensor readings
        while (System.currentTimeMillis() < stop) {
            count++;
            total += getAlpha();
        }

        return total / count > alphaThreshold; //return true if our readings were high enough to signify a ball being sensed
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

    //close shooter door
    public void close() {
        door.setPosition(doorPositions[1]);
    }

    //open shooter door
    private void open() {
        door.setPosition(doorPositions[0]);
    }

    public double getAlpha() {
        return Utils.getMagnitude(ballSensor.red(), ballSensor.blue());
    }

}
