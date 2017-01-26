package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Sensors sensors;
    private Shooter shooter;

    private final boolean isRed;

    private final double odsThresholdFindButton = 0.04;

    private final double odsThresholdPushButton = 0.1;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed, LinearOpMode opMode) {
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.isRed = isRed;

        sensors.initImu();
    }

    public void run() {
        shooter.shoot(6);

        if (isRed) {
            //must drive forward a hair to clear wall
            sensors.turnAround();
        }

        sensors.driveUntilLineReadingThreshold(Math.PI / 4 * (isRed ? -1 : 1), whiteLineSignalThreshold);

        captureBeacon();

        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold);

        sensors.centerOnZero();

        captureBeacon();
    }

    private void captureBeacon() {
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton); //pull up to beacon

        sensors.findBeaconButton(isRed); //align on button

        sensors.driveUntilOdsThreshold(0, odsThresholdPushButton, true); //press button

        sensors.driveUntilOdsThreshold(Math.PI, odsThresholdFindButton, false); //pull away from beacon

        sensors.centerOnZero(); //align orientation
    }

}