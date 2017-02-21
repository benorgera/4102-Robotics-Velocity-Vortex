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
    private final double odsRealignThreshold = 0.046;


    private final double whiteLineSignalThreshold = 60; //the minimum color sensor reading required to signify finding the white line


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

        Hardware.print("Moving away from wall");
        sensors.driveByTime(-Math.PI / 2, 250, true, 0.3);

        Hardware.print("Shooting");
        shooter.shoot(6.64);

        if (isRed) {
            Hardware.print("Pulling away from wall");
            sensors.driveByTime(-Math.PI / 2, 300, true);
            Hardware.print("Turning around");
            sensors.turn(-Math.PI, Math.PI / 16, 0.4);
        }

        Hardware.print("Picking up momentum to find line");
        sensors.driveByTime(isRed ? 1 : -1 * Math.PI / 2, 300, false, 0.35);

        Hardware.print("Finding first beacon line");
        sensors.driveUntilLineReadingThreshold(isRed ? (8 * Math.PI / 9) : (13 * Math.PI / 12), whiteLineSignalThreshold, true, true, 0, 10000, 0.4); //translate to line in front of first beacon

        Hardware.sleep(200);

        Hardware.print("Capturing first beacon");
        captureBeacon();

        Hardware.print("Driving by time to second beacon");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 800, false, 0.4);

        Hardware.print("Finding second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, true, 0, 2200, 0.4); //translate to line in front of second beacon

        Hardware.sleep(200);

        Hardware.print("Realigning on second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, false, true, 0, 10000, 0.17);

        Hardware.print("Capturing second beacon");
        captureBeacon();

        Hardware.print("Turning towards the cap ball");
        sensors.turn(3 * Math.PI / 14 * (isRed ? 1 : -1), Math.PI / 20, 0.4);

        Hardware.getIntake().moveRampForShot();

        Hardware.print("Driving to the cap ball");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1600, true, 1);
    }

    private void captureBeacon() {

        Hardware.print("Following line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, 4000, false, 0.15); //pull up to beacon

        Hardware.sleep(250);

        realignOnBeacon();

        if (isDoublePushing) {
            pushButton();
            long readyTime = System.currentTimeMillis() + 5000; //5 second delay on beacons

            Hardware.sleep(500); //just in case the color change takes time

            if (sensors.getBeaconColor()[isRed ? 0 : 1] > sensors.getBeaconColor()[isRed ? 1 : 0]) { //we need to push again
                realignOnBeacon();

                if (readyTime > System.currentTimeMillis())
                    Hardware.sleep(readyTime - System.currentTimeMillis());

                pushButton();
            }
        } else {
            Hardware.print("Finding button");

            if (sensors.findBeaconButton(isRed, whiteLineSignalThreshold, 7000, 0.13)) {
                Hardware.sleep(500);
                pushButton();
            }
        }

        backUpFromBeacon();
    }

    private void realignOnBeacon() {
        Hardware.print("Realigning on beacon");
        sensors.followLineUntilOdsThreshold(odsRealignThreshold, 3000, true, 0.125);
    }

    private void backUpFromBeacon() {
        Hardware.print("Backing up");
        sensors.driveByTime(0, 600, true);
    }

    private void pushButton() {
        Hardware.print("Pushing button");

        for (int i = 0; i < 4; i++) {
            sensors.driveByTime(Math.PI, 100, true, 1);
            Hardware.sleep(100);
        }

        sensors.driveByTime(Math.PI, 400, true, 1);
    }
}