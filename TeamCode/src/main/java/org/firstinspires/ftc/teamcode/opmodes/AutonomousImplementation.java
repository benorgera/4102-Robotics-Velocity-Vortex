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

    private final double odsThresholdFindButton = 0.0485;

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
        Hardware.print("About to shoot");
        shooter.shoot(6);

        if (isRed) {
            Hardware.print("isRed = true");
            sensors.driveByTime(Math.PI / 2, 2000, true);
            sensors.turnAround();
            sensors.centerOnZero();
        }

        Hardware.print("About to pick up momentum to find line");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 300, false, 0.35);

        Hardware.print("About to find first beacon line");
        sensors.driveUntilLineReadingThreshold(isRed ? (5 * Math.PI / 6) : (9 * Math.PI / 8), whiteLineSignalThreshold, true, 0.35); //translate to line in front of first beacon

        Hardware.print("About to stop and lose momentum");
        Utils.sleep(500);

        Hardware.print("About to capture first beacon");
        captureBeacon();

        Hardware.print("About to drive by time to second beacon");
        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 2000, false, 0.35);

        Hardware.print("About to find second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, 0.35); //translate to line in front of second beacon

        Hardware.print("About to stop and lose momentum");
        Utils.sleep(500);

        Hardware.print("About to realign on second beacon line");
        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, false, 0.17);

        Hardware.print("About to capture second beacon");
        captureBeacon();

        Hardware.print("About to drive to the cap ball");
        sensors.driveByTime(isRed ? Math.PI : Math.PI / 7, 4000, true, 1);
    }

    private void captureBeacon() {

        Hardware.print("About to follow line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, true); //pull up to beacon

        Hardware.print("About to find button");
        sensors.findBeaconButton(isRed, 0.135);

        Hardware.print("About to push button");
        Utils.sleep(500);
        sensors.driveByTime(Math.PI, 700, true);

        Hardware.print("Pushed beacon button, about to back up");
        sensors.driveByTime(0, 600, true);
    }

}