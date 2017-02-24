package org.firstinspires.ftc.teamcode.components;

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

    private final double[] doorPositions = {0.333, 1}; //open and closed respectively

    private final double alphaThreshold = 100;

    public Shooter(DcMotor[] disks, Servo door, ColorSensor ballSensor) {
        this.door = door;
        this.disks = disks;
        this.ballSensor = ballSensor;

        door.setPosition(doorPositions[1]);

        for (DcMotor m : disks) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ((ModernRoboticsUsbDcMotorController) m.getController()).setDifferentialControlLoopCoefficients(m.getPortNumber(), new DifferentialControlLoopCoefficients(160, 32, 112));
            ((ModernRoboticsUsbDcMotorController) m.getController()).setGearRatio(m.getPortNumber(), 25);
        }

        disks[1].setDirection(DcMotor.Direction.REVERSE);
    }

    public void shoot(double speed) {
        ballSensor.enableLed(true); //turn on LED so we can sense the ball
        Hardware.getWheels().stop(); //stop the robot in case its moving
        door.setPosition(doorPositions[0]); //open shooter

        setDiskMotorPowers(speed / 10); //bring motors up to speed by starting the PID

        Hardware.sleep(1500); //wait for motors to come up to speed

        while (takeShot()); //while there's another ball remaining, shoot

        //reset stuff
        ballSensor.enableLed(false);
        Hardware.getIntake().stopElevator();
        stop(); //stop the PID
        door.setPosition(doorPositions[1]); //reset ramp for next intaking
        Hardware.getIntake().dropRamp(0);
    }

    //take shot while regulating speed, stopping elevator after first ball passes through
    //return true if another ball remains, and therefore another shot should be taken
    private boolean takeShot() {
        Hardware.getIntake().moveRampForShot(); //move ramp out of way of intake
        Hardware.getIntake().startElevator(); //feed shot through the shooter

        //wait for the next ball to be in position
        //if this command times out, false is returned and no balls remain so no more shots should be taken
        boolean takingAnotherShot = waitForNextBall();

        if (takingAnotherShot) Hardware.sleep(200); //wait for PID to attain desired speed again

        Hardware.getIntake().stopElevator(); //stop elevator once next ball is sensed, or times out

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
        while (!hasBall() || System.currentTimeMillis() - lastHadBall < 150 && (foundAnotherBall = System.currentTimeMillis() - lastHadBall < 200))
            Hardware.sleep(10);

        return foundAnotherBall; //return true unless no additional balls were found
    }

    private void setDiskMotorPowers(double power) {
        for (DcMotor m : disks)
            m.setPower(power);
    }

    //returns true if a ball is sensed in the elevator, ready to be shot
    private boolean hasBall() {
        return ballSensor.alpha() > alphaThreshold;
    }

    public void stop() {
        setDiskMotorPowers(0);
    }

    public int getAlpha() {
        return ballSensor.alpha();
    }

}
