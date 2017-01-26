package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Wheels wheels;
    private Sensors sensors;
    private Shooter shooter;
    private Intake intake;

    private LinearOpMode opMode;

    private final boolean isRed;

    private final double odsThresholdFindButton = 0.04;

    private final double odsThresholdPushButton = 0.1;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed, LinearOpMode opMode) {
        Hardware.setIsAuton(true);
        this.wheels = Hardware.getWheels();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.intake = Hardware.getIntake();
        this.opMode = opMode;

        this.isRed = isRed;

        sensors.initImu();
        sensors.setOpMode(opMode);
    }

    public void run() {
        shooter.shoot(6);

        if (isRed) {

            sensors.turnAround();
        }

        sensors.driveUntilLineReadingThreshold(Math.PI / 4 * (isRed ? -1 : 1), whiteLineSignalThreshold);

        captureBeacon();

        sensors.driveUntilLineReadingThreshold(Math.PI / 2 * (isRed ? -1 : 1), whiteLineSignalThreshold);

        sensors.centerOnZero();

        captureBeacon();
    }

    private void captureBeacon() {
        sensors.followLineUntilOdsThreshold(odsThresholdFindButton);

        sensors.findBeaconButton(isRed);

        sensors.driveUntilOdsThreshold(0, odsThresholdPushButton);

        sensors.centerOnZero();
    }

}