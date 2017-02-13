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

    private final double odsThresholdFindButton = 0.025;
    private final double odsRealignThreshold = 0.055;

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
        sensors.driveUntilLineReadingThreshold(isRed ? (9 * Math.PI / 10) : (8 * Math.PI / 7), whiteLineSignalThreshold, true, 0.4); //translate to line in front of first beacon

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

        Hardware.print("About to capture second beacon");
        captureBeacon();

        Hardware.print("About to turn towards the cap ball");
        sensors.turn(Math.PI / 4 * (isRed ? -1 : 1), 0.4);

        Hardware.getIntake().moveRampForShot();

        Hardware.print("About to drive to the cap ball");
        sensors.driveByTime(Math.PI / 2 * (isRed ? -1 : 1), 1600, true, 1);
    }

    private void captureBeacon() {

        Hardware.print("About to follow line");
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, true, 0.15); //pull up to beacon

        Hardware.sleep(500);

        Hardware.print("About to realign on beacon");
        sensors.driveUntilOdsThreshold(odsRealignThreshold, true, 0.09);

        Hardware.print("About to find button");
        sensors.findBeaconButton(isRed, whiteLineSignalThreshold, 0.155);

        Hardware.print("About to push button");
        Hardware.sleep(500);
        sensors.driveByTime(Math.PI, 700, true);

        Hardware.print("Pushed beacon button, about to back up");
        sensors.driveByTime(0, 600, true);
    }

}