package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Sensors sensors;
    private Shooter shooter;

    private final double thetaToWall = 2 * Math.PI / 11;
    private final boolean isRed;

    public AutonomousImplementation(boolean isRed) { //initializes all of our robot's component, noting our alliance and whether we are running a double push autonomous
        Hardware.getLift();
        Hardware.getIntake();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.isRed = isRed;

        sensors.initImu(); //start up the adafruit imu
    }

    public void run() {
        Hardware.print("Color is " + (isRed ? "red" : "blue"));

        Hardware.print("Prepping for shot");
        shooter.prepShot(6.57); //speeds up the wheels

        Hardware.print("Moving away from wall");
        sensors.driveByTime(-Math.PI / 2, 270, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall

        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(1500); //allows the shooting motors to finish getting to the right speed

        Hardware.print("Shooting");
        shooter.shoot(0); //shoots the ball at the same prepshot speed

        Hardware.print("Pulling away for turn");
        sensors.driveByTime(-Math.PI / 2, 300, true);

        Hardware.print("Turning towards wall");
        sensors.turn(isRed ? thetaToWall - Math.PI : -thetaToWall, isRed ? Math.PI / 13 : Math.PI / 30, 0.4);

        Hardware.print("Driving to wall");
        sensors.driveUntilLineOrTouchOrRange(0.2, 0.09, isRed, 40);

        Hardware.print("Parallel Parking");
        sensors.parallelPark(Math.PI / 2 * (isRed ? 1 : -1), thetaToWall * (isRed ? 1 : -1), 0.3, Math.PI / 12, thetaToWall * (isRed ? -1 : 1), 0.3, Math.PI / 25);

        hugWall();

        //drive to and capture each beacon
        for (int i = 0; i < 2; i++) {
            Hardware.print("Finding first beacon line"); //drives to the beacon line
            driveToLine((i == 0) == isRed);

            hugWall();

            Hardware.print("Capturing first beacon");
            sensors.captureBeacon(isRed); //press proper button
        }

        Hardware.print("Backing up from wall");
        sensors.driveByTime(0, 1000, true, 1);

        Hardware.print("Partial Parking");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1000, true, 1);
    }

    private void driveToLine(boolean intakeForward) {
        Hardware.print("Drive to line");
        long realignTimeout = 2000;

        if (!sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? 1 : -1), false, true, 750, 2000, 0.25, 40)) {
            realignTimeout += 1000;
            Hardware.print("Initial line drive timeout");
        }

        Hardware.print("Realign on line");

        while (!sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? -1 : 1), false, true, 200, realignTimeout, 0.12, 90)) {
            Hardware.print("Realign timeout after " + realignTimeout + " ms");
            intakeForward = !intakeForward;
            realignTimeout += 1000;
        }
    }

    private void hugWall() {
        sensors.driveUntilOdsThreshold(Math.PI, 0.15, 0.5, 500);
    }

}
