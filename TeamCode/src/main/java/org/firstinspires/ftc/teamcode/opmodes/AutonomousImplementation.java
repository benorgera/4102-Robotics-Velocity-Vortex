package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Utils;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Sensors sensors;
    private Shooter shooter;

    private final boolean isRed;

    private final double odsThresholdFindButton = 0.045;

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
        Hardware.print("Starting shot");
        shooter.shoot(10);
        Hardware.print("Finished shot");

        if (isRed) {
            Hardware.print("isRed = true");
            sensors.driveByTime(Math.PI / 2, 2000, true);
            sensors.turnAround();
            sensors.centerOnZero();
        }

        Hardware.print("About to find line");

        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 600, false);

        sensors.driveUntilLineReadingThreshold(isRed ? (5 * Math.PI / 6) : (7 * Math.PI / 6), whiteLineSignalThreshold, false, true); //translate to line in front of first beacon

        Hardware.print("Found line, about to capture beacon");

        captureBeacon();

        Hardware.print("About to translate to second beacon");

        sensors.driveByTime(Math.PI / 2 * (isRed ? 1 : -1), 2000, false);

        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? 1 : -1), whiteLineSignalThreshold, false, true); //translate to line in front of second beacon

        Utils.sleep(1000);

        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold, true, false);

        Hardware.print("Translated to second beacon, about to capture beacon");

        captureBeacon();
    }

    private void captureBeacon() {

        Hardware.print("About to follow line");

        sensors.followLineUntilOdsThreshold(odsThresholdFindButton, true); //pull up to beacon

        Hardware.print("In front of beacon, about to find button");

        sensors.findBeaconButton(isRed); //align on button

        Hardware.print("Found beacon button, about to push button");

        Utils.sleep(500);

        sensors.driveByTime(Math.PI, 700, true); //press button

        Hardware.print("Pushed beacon button, about to back up");

        sensors.driveByTime(0, 700, true); //pull away from beacon

//        sensors.driveUntilOdsThreshold(0, odsThresholdFindButton, false, false); //pull away from beacon

        Hardware.print("Backed up from button, about to center on zero");
    }

}