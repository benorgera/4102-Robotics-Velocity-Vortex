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

    private final double odsThreshold = 0.04;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed, LinearOpMode opMode) {
        this.wheels = Hardware.getWheels();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.intake = Hardware.getIntake();
        this.opMode = opMode;

        this.isRed = isRed;

        sensors.initImu();
    }

    public void run() {
        shooter.shoot(0.6);

        if (isRed) sensors.turnAround();

        driveUntilLine(Math.PI / 4);

        wheels.stop();

        followLine();

        wheels.stop();

        sensors.findBeaconButton(isRed, opMode);

        pushButton();

        sensors.centerOnZero(opMode);

        driveUntilLine(Math.PI / 2);

        wheels.stop();

        sensors.centerOnZero(opMode);

        followLine();

        wheels.stop();

        sensors.findBeaconButton(isRed, opMode);

        pushButton();

        sensors.centerOnZero(opMode);
    }

    private void pushButton() {
        wheels.drive(0.35, 0, 0, false);
        Utils.sleep(500);
        wheels.stop();
    }

    private void followLine() {
        //drive until we reach the beacon
        sensors.followLine(true);
        while (opMode.opModeIsActive() && sensors.getOpticalDistance() < odsThreshold)
            sensors.followLine(false);
    }

    private void driveUntilLine(double theta) {
        sensors.compensatedTranslate(theta * (isRed ? -1 : 1), true, false);
        while (opMode.opModeIsActive() && Utils.getMaxMagnitude(sensors.getLineReadings()) < whiteLineSignalThreshold)
            sensors.compensatedTranslate(theta * (isRed ? -1 : 1), false, false);
    }

}