package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

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

        boolean hasShot = false;

        while (takeShot(hasShot) && Hardware.active()) hasShot = isAuton; //while there's another ball remaining, shoot

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

        Hardware.print(takingAnotherShot ? "taking" : "not taking" + " another shot");

        Hardware.print("stop elevator");

        if (takingAnotherShot) {
            Hardware.getIntake().runElevator(-0.3);
            Hardware.sleep(20);
            Hardware.getIntake().stopElevator();;
            Hardware.sleep(600);
        } else {
            Hardware.getIntake().stopElevator();
        }

        return takingAnotherShot; //return true if another shot is to be taken
    }

    //wait for the next ball to be sensed, and return true unless another ball is never found
    private boolean waitForNextBall(boolean isLastShot) {

        long started = System.currentTimeMillis();

        Hardware.print("while has ball");
        while (hasBall() && Hardware.active()); //wait for the ball currently being shot to pass through

        long lastHadBall = System.currentTimeMillis();

        Hardware.print("had ball for " + (lastHadBall - started) + " ms");

        if (isLastShot) return false;

        //continue running the elevator (waiting) until you see a new ball, or you time out
        //waiting will only cease after at least 50ms have passed since last sensing a ball
        Hardware.print("while not next ball");
        while (Hardware.active() && !hasBall() && (System.currentTimeMillis() - lastHadBall) < 1000 || System.currentTimeMillis() - lastHadBall < 150);

        Hardware.print("waited " + (System.currentTimeMillis() - lastHadBall) + "ms for next ball");
        return System.currentTimeMillis() - lastHadBall < 500; //return true unless no additional balls were found
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    //returns true if a ball is sensed in the elevator, ready to be shot
    private boolean hasBall() {
        long stop = System.currentTimeMillis() + 75;

        double total = 0;
        int count = 0;

        while (System.currentTimeMillis() < stop) {
            count++;
            total += getAlpha();
        }

        Hardware.print("avg: " + total / count);
        Hardware.print("c: " + count);

        return total / count > alphaThreshold;
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

    public void close() {
        door.setPosition(doorPositions[1]);
    }

    private void open() {
        door.setPosition(doorPositions[0]);
    }

    public double getAlpha() {
        return Utils.getMagnitude(ballSensor.red(), ballSensor.blue());
    }

}
