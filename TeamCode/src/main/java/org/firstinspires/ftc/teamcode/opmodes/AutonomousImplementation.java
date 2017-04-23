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

    private final double thetaToWall = Math.PI / 5;
    private final boolean isRed;
    private final boolean isParkingCenter;

    public AutonomousImplementation(boolean isRed, boolean isParkingCenter) { //initializes all of our robot's component, noting our alliance
        Hardware.getLift();
        Hardware.getIntake();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.isRed = isRed;
        this.isParkingCenter = isParkingCenter;


        long start = System.currentTimeMillis();

        Hardware.print("Starting fruit");
        sensors.initImu(); //start up the adafruit imu
        Hardware.print("Stopping fruit, init in " + (System.currentTimeMillis() - start));
    }

    public void run() {
        Hardware.print("Color is " + (isRed ? "red" : "blue"));

        Hardware.print("Prepping for shot");
        shooter.prepShot(6.72); //speeds up the wheels

        Hardware.print("Moving away from wall");
        sensors.driveByTime(-Math.PI / 2, 400, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall

        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(2500); //allows the shooting motors to finish getting to the right speed

        Hardware.print("Shooting");
        shooter.shoot(700); //shoots the ball at the same prepshot speed

        Hardware.print("Turning towards wall");
        sensors.turn(isRed ? thetaToWall - Math.PI : -thetaToWall, isRed ? Math.PI / 13 : Math.PI / 18, 0.4);

        Hardware.print("Driving to wall");
        sensors.driveUntilLineOrTouchOrRange(0.26, 0.105, isRed, 1100, 3500, 7000, 40, 65);

        Hardware.disableRangeSensors(); //range sensors no longer needed

        Hardware.print("Parallel Parking");
        sensors.parallelPark(Math.PI / 2 * (isRed ? 1 : -1), thetaToWall * (isRed ? 1 : -1), 0.3, Math.PI / 12, thetaToWall * (isRed ? -1 : 1), 0.3, Math.PI / 25, 2500);

        //drive to and capture each beacon
        for (int i = 0, max = 2; i < max; i++) {
            hugWall();

            Hardware.print("Finding beacon line"); //drives to the beacon line
            driveToLine((i == 0 || i == 3) == isRed, i == 0);

            hugWall();

            Hardware.print("Capturing beacon");
            if (!sensors.captureBeacon(isRed) && i > 0) { //push button
                Hardware.print("Rediscovered first beacon, running again"); //found the same beacon a second time, run again to claim the second beacon
                max++;
            }
        }

        Hardware.print("Backing up from wall");
        sensors.driveByTime(0, isParkingCenter ? 700 : 1000, true, 1);

        if (isParkingCenter) {
            Hardware.print("Turning towards center vortex");
            sensors.turn(Math.PI / 2 + (2 * Math.PI / 17 * (isRed ? 1 : -1)), Math.PI / 10, 0.6);
        }

        Hardware.print(isParkingCenter ? "Knocking Cap Ball" : "Partial parking");
        sensors.driveByTime(Math.PI / 2 * (isParkingCenter || isRed ? -1 : 1), 1000, true, 1);

        if (isParkingCenter) {
            Hardware.print("Partial Parking");
            sensors.turn(Math.PI / 15 * (isRed ? -1 : 1), Math.PI / 90, 1);

            sensors.driveByTime(-Math.PI / 2, 550, true, 1);
        }
    }

    private void driveToLine(boolean intakeForward, boolean isFirstBeacon) {
        Hardware.print("Driving to line");

        long minTime = isFirstBeacon ? 0 : 1200, //a minimum drive time to ensure the previous line isn't rediscovered
                timeout = isFirstBeacon ? 1500 : 4000;
        double power = 0.21;

        boolean hasTimedOut = false;

        //drive to the line until it is found
        while (!sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? 1 : -1), false, true, minTime, timeout, power, 60, 4)) {
            if (hasTimedOut) { //multiple timeouts, drive for longer and switch direction
                timeout += 1000;
            } else { //first timeout, begin back and forth algorithm with a short drive
                timeout = 1500;
                hugWall();
            }
            power = 0.175; //a timeout occurred, slow down
            minTime = 0; //time has been spent, no need to worry about finding the same line again
            timeout += 1000; //increase the time allotted to find the line
            intakeForward = !intakeForward; //switch direction, the line has likely been overshot
            hasTimedOut = true;
            Hardware.print("Line drive timeout, timeout now " + timeout);
        }

        hugWall();

        //drive the opposite direction (we've presumably drifted past the line), slower and looking for a stronger reading, to ensure alignment
        Hardware.print("Realigning on line");
        if (!sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? -1 : 1), false, true, 200, 1600, 0.12, 90, 4)) {
            Hardware.print("Realign timeout, rerunning drive method");
            driveToLine(isFirstBeacon == isRed, isFirstBeacon);
        }
    }

    private void hugWall() {
        sensors.driveUntilOdsThreshold(Math.PI, 0.3, 4, 0.5, 1000);
    }

}
