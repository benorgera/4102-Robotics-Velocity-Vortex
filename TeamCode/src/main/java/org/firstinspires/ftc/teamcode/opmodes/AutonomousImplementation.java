package org.firstinspires.ftc.teamcode.opmodes;

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

    private final double odsThresholdFindButton = 0.048;
    private final double odsRealignThreshold = 0.1;

    private final double whiteLineSignalThreshold = 70; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed) {
        Hardware.getLift();
        Hardware.getIntake();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.isRed = isRed;

        sensors.initImu();
    }

    public void run() {
        Hardware.print("Color is " + (isRed ? "red" : "blue"));

        Hardware.print("About to shoot");
        shooter.shoot(6.1);

        if (isRed) {
            Hardware.print("About to pull away from wall");
            sensors.driveByTime(-Math.PI / 2, 600, true);
            Hardware.print("About to turn around");
            sensors.turn(Math.PI, 0.29);
        }

        Hardware.print("About to pick up momentum to find line");
        sensors.driveByTime(isRed ? 1 : -1 * Math.PI / 2, 300, false, 0.35);

        Hardware.print("About to find first beacon line");
        sensors.driveUntilLineReadingThreshold(isRed ? (6 * Math.PI / 7) : (9 * Math.PI / 8), whiteLineSignalThreshold, true, 0.35); //translate to line in front of first beacon

        Hardware.print("About to stop and lose momentum");
        Hardware.sleep(500);

        Hardware.print("About to capture first beacon");
        captureBeacon();

        Hardware.print("About to drive by time to second beacon");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 600, false, 0.35);

        Hardware.print("About to find second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, 0.35); //translate to line in front of second beacon

        Hardware.print("About to stop and lose momentum");
        Hardware.sleep(500);

        Hardware.print("About to realign on second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, false, 0.17);

        Hardware.print("About to capture second beacon");
        captureBeacon();

        Hardware.print("About to drive to the cap ball");
        sensors.driveByTime((isRed ? -1 : 1) * Math.PI / 7, 4000, true, 1);

        Hardware.print("About to turn");
        sensors.turn(Math.PI / 2, 1);
    }

    private void captureBeacon() {

        Hardware.print("About to follow line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, true, 0.22); //pull up to beacon

        Hardware.sleep(500);

        Hardware.print("About to realign on beacon");
        sensors.driveUntilOdsThreshold(odsRealignThreshold, true, 0.108);

        Hardware.print("About to find button");
        sensors.findBeaconButton(isRed, whiteLineSignalThreshold, 0.115);

        Hardware.print("About to push button");
        Hardware.sleep(500);
        sensors.driveByTime(Math.PI, 700, true);

        Hardware.print("Pushed beacon button, about to back up");
        sensors.driveByTime(0, 600, true);
    }

}