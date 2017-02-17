package org.firstinspires.ftc.teamcode.opmodes;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.utilities.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.utilities.Utils;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Sensors sensors;
    private Shooter shooter;

    private final boolean isRed;
    private final boolean isDoublePushing;

    private final double odsThresholdFindButton = 0.03;
    private final double odsRealignThreshold = 0.05;

    private final double whiteLineSignalThreshold = 62; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed, boolean isDoublePushing) {
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

        Hardware.print("About to move away from wall");
        sensors.driveByTime(-Math.PI / 2, 150, true, 0.3);

        Hardware.print("About to shoot");
        shooter.shoot(6.6);

        if (isRed) {
            Hardware.print("About to pull away from wall");
            sensors.driveByTime(-Math.PI / 2, 600, true);
            Hardware.print("About to turn around");
            sensors.turn(Math.PI, 0.4);
        }

        Hardware.print("About to pick up momentum to find line");
        sensors.driveByTime(isRed ? 1 : -1 * Math.PI / 2, 300, false, 0.35);

        Hardware.print("About to find first beacon line");
        sensors.driveUntilLineReadingThreshold(isRed ? (9 * Math.PI / 10) : (10 * Math.PI / 9), whiteLineSignalThreshold, true, 0.4); //translate to line in front of first beacon

        Hardware.print("About to stop and lose momentum");
        Hardware.sleep(500);

        Hardware.print("About to capture first beacon");

        captureBeacon();

        Hardware.print("About to drive by time to second beacon");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 1000, false, 0.25);

        Hardware.print("About to find second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, 0.25); //translate to line in front of second beacon

        Hardware.print("About to stop and lose momentum");
        Hardware.sleep(500);

        Hardware.print("About to realign on second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, false, 0.17);

        Hardware.sleep(500);

        Hardware.print("About to capture second beacon");
        captureBeacon();

        Hardware.print("Turning towards the cap ball");
        sensors.turn(Math.PI / 4 * (isRed ? -1 : 1), 0.4);

        Hardware.getIntake().moveRampForShot();

        Hardware.print("Driving to the cap ball");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1600, true, 1);
    }

    private void captureBeacon() {

        Hardware.print("Following line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, 0.15); //pull up to beacon

        Hardware.sleep(500);

        realignOnBeacon();

        if (isDoublePushing) {
            pushButton();
            long readyTime = System.currentTimeMillis() + 5000; //5 second delay on beacons

            Hardware.sleep(2000); //just in case the color change takes time

            if (sensors.getBeaconColor()[isRed ? 0 : 1] > sensors.getBeaconColor()[isRed ? 1 : 0]) { //we need to push again
                realignOnBeacon();

                if (readyTime > System.currentTimeMillis())
                    Hardware.sleep(readyTime - System.currentTimeMillis());

                pushButton();
            }
        } else {
            Hardware.print("Finding button");
            sensors.findBeaconButton(isRed, whiteLineSignalThreshold, 0.115);

            Hardware.sleep(500);

            pushButton();
        }

        backUpFromBeacon();
    }

    private void realignOnBeacon() {
        Hardware.print("Realigning on beacon");
        sensors.followLineUntilOdsThreshold(odsRealignThreshold, 0.1);
    }

    private void backUpFromBeacon() {
        Hardware.print("Backing up");
        sensors.driveByTime(0, 600, true);
    }

    private void pushButton() {
        Hardware.print("Pushing button");
        sensors.driveByTime(Math.PI, 800, true, 1);
    }

}