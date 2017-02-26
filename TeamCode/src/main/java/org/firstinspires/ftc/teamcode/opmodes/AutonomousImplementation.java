package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation { //this is the basic template for autonomous implementation. all other autonomouses reference this

    private Sensors sensors;
    private Shooter shooter;

    private final boolean isRed;
    private final boolean isDoublePushing;

    private final double odsThresholdFindButton = 0.03;
    private final double odsRealignThreshold = 0.049;


    private final double whiteLineSignalThreshold = 60; //the minimum color sensor reading required to signify finding the white line

    public AutonomousImplementation(boolean isRed, boolean isDoublePushing) { //initializes all of our robot's components. also checks to see if we are red or not, and whether we are running a double push autonomous
        Hardware.getLift();
        Hardware.getIntake();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.isRed = isRed;
        this.isDoublePushing = isDoublePushing;

        sensors.initImu();
    }

    public void run() {
        Hardware.print("Color is " + (isRed ? "red" : "blue"));

        Hardware.print("Prepping for shot");
        shooter.prepShot(6.64); //speeds up the wheels

        Hardware.print("Moving away from wall");
        sensors.driveByTime(-Math.PI / 2, 200, false, 0.3); //drives a short distance from the wall so our intake is not slowed by hitting the wall

        Hardware.getWheels().softStop(300); //stops the robot gently to avoid jerk when launching the balls
        Hardware.sleep(1000); //allows the shooting motors to finish getting to the right speed

        Hardware.print("Shooting");
        shooter.shoot(6.64, true); //shoots the ball at the same prepshot speed

        if (isRed) { //if we are red, our robot drives out a bit further and turns around so the button pusher is on the correct side
            Hardware.print("Pulling away from wall");
            sensors.driveByTime(-Math.PI / 2, 300, true);
            Hardware.print("Turning around");
            sensors.turn(-Math.PI, Math.PI / 13, 0.4);
        }

        Hardware.print("Picking up momentum to find line"); //makes sure we have the momentum to drive to the beacon and not get stuck
        sensors.driveByTime(isRed ? 1 : -1 * Math.PI / 2, 300, false, 0.35);

        Hardware.print("Finding first beacon line"); //drives to the beacon line
        sensors.driveUntilLineReadingThreshold(isRed ? (16 * Math.PI / 17) : (9 * Math.PI / 8), whiteLineSignalThreshold, true, true, 0, 10000, 0.4); //translate to line in front of first beacon

        Hardware.sleep(200);

        Hardware.print("Capturing first beacon");
        captureBeacon(); //follows the line, and presses the correct beacon button– function below

        Hardware.print("Driving by time to second beacon");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 800, false, 0.4); //translates to the next beacon

        Hardware.print("Finding second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, true, 0, 1600, 0.4); //translate to line in front of second beacon

        Hardware.sleep(200);

        realignOnLine(); //compensates for the robot's momentum when driving to the second line

        Hardware.print("Capturing second beacon");
        captureBeacon(); //follows line and presses correct beacon button

        Hardware.print("Turning towards the cap ball");
        sensors.turn(2 * Math.PI / 9 * (isRed ? 1 : -1), Math.PI / 30, 0.4); //points itself towards the ball by the center vortex

        Hardware.getIntake().moveRampForShot(); //moves the ramp up so it doesn't hit the center vortex and break itself

        Hardware.print("Driving to the cap ball");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1600, true, 1); //knocks over the cap ball and partial parks
    }

    private void captureBeacon() {

        Hardware.print("Following line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, 4000, false, 0.165); //pull up to beacon

        Hardware.sleep(250);

        realignOnBeacon(); //makes sure we are aligned on the beacon

        if (isDoublePushing) { //if it is running the double push autonomous, it presses a button, waits and checks whether the color is correct, and if not it presses again.
            pushButton();
            long readyTime = System.currentTimeMillis() + 5000; //5 second delay on beacons

            Hardware.sleep(500); //just in case the color change takes time

            if (sensors.getBeaconColor()[isRed ? 0 : 1] > sensors.getBeaconColor()[isRed ? 1 : 0]) { //we need to push again
                realignOnBeacon(); //realigns to beacon

                if (readyTime > System.currentTimeMillis()) //makes sure we have waited long enough, if not then it waits the remaining time
                    Hardware.sleep(readyTime - System.currentTimeMillis());

                pushButton(); //pushes the button
            }
        } else { //if we are running the normal autonomous, it finds the correct beacon button and presses it
            Hardware.print("Finding button");

            if (sensors.findBeaconButton(isRed, whiteLineSignalThreshold, 7000, 0.13)) { //finds the correct beacon button based on sensor values
                Hardware.sleep(500);
                pushButton(); //pushes said button
            }
        }

        backUpFromBeacon(); //backs up because it will be too close for a translate/rotate
    }

    private void realignOnBeacon() { //makes sure we are the correct distance from the beacon
        Hardware.print("Realigning on beacon");
        sensors.followLineUntilOdsThreshold(odsRealignThreshold, 3000, true, 0.135);
    }

    private void backUpFromBeacon() { //backs away a little bit
        Hardware.print("Backing up");
        sensors.driveByTime(0, 600, true);
    }

    private void pushButton() { //drives forward to press the button
        Hardware.print("Pushing button");

        for (int i = 0; i < 5; i++) { //incrementally drives forward five times in order to make sure the robot drives straigt
            sensors.driveByTime(Math.PI, 100, true, 1);
            Hardware.sleep(100);
        }

        sensors.driveByTime(Math.PI, 400, true, 1); //drives the rest of the way forward to really press the button
    }

    private void realignOnLine() { //realigns on the beacon line in case we pass it
        Hardware.print("Realigning on beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, false, true, 0, 10000, 0.17);
    }
}
