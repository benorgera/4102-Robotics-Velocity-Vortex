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

    private final double thetaToWall = 2 * Math.PI / 9;
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
        shooter.prepShot(6.2); //speeds up the wheels

        Hardware.print("Moving away from wall");
        sensors.driveByTime(-Math.PI / 2, 200, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall

        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(1500); //allows the shooting motors to finish getting to the right speed

        Hardware.print("Shooting");
        shooter.shoot(1000); //shoots the ball at the same prepshot speed

        Hardware.print("Pulling away for turn");
        sensors.driveByTime(-Math.PI / 2, 300, true);

        Hardware.print("Turning towards wall");
        sensors.turn(isRed ? thetaToWall - Math.PI : -thetaToWall, isRed ? Math.PI / 13 : Math.PI / 30, 0.4);

        Hardware.print("Driving to wall");
        if (!sensors.driveUntilLineOrTouchOrRange(0.3, isRed))
            sensors.driveUntilTouchReading(0.09, isRed);

        Hardware.print("Parallel Parking");
        sensors.parallelPark(Math.PI / 2 * (isRed ? 1 : -1), thetaToWall * (isRed ? 1 : -1), 0.25, Math.PI / 15, thetaToWall * (isRed ? -1 : 1), 0.3, Math.PI / 25);

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
        sensors.driveByTime(0, 500, true, 1);

        Hardware.print("Partial Parking");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1000, true, 1);
    }

    private void driveToLine(boolean intakeForward) {
        Hardware.print("Drive to line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? 1 : -1), false, true, 500, 7000, 0.4);

        Hardware.print("Realign on line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (intakeForward ? -1 : 1), false, true, 200, 1000, 0.17);
    }

    private void hugWall() {
        sensors.driveByTime(Math.PI, 200, true, 0.4);
    }

}
