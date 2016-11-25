package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    private Stage s = Stage.SHOOTING;

    private double initialHeading;

    private final boolean isRed;

    private boolean isActive = true;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line

    public AutonomousImplementation(boolean isRed) {
        this.wheels = Hardware.getWheels();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.intake = Hardware.getIntake();
        this.isRed = isRed;
    }


    public void loop() {
        switch (s) {
            case SHOOTING:
                shoot();
                break;
            case FINDING_FIRST_WHITE_LINE:
                findFirstWhiteLine();
                break;
            case PUSHING_BUTTON:
                break;
            case KNOCKING_BALL:
                break;
            case PARKING:
                break;
        }
    }

    private void shoot() {
        //get the shooters up to speed
        shooter.run(1);
        Utils.sleep(1500);

        //take the shot
        intake.run();
        Utils.sleep(1500);

        //stop the intake and shooter
        intake.stop();
        shooter.stop();
        Utils.sleep(1500);

        sensors.centerIMU();
        intake.holdIMU();
        Utils.sleep(1500);
        sensors.initImu();

        initialHeading = sensors.getHeading();
        s = Stage.FINDING_FIRST_WHITE_LINE;

    }

    private void findFirstWhiteLine() {
        double[][] readings = sensors.getLineData();
        boolean foundLine = false;

        for (double[] a : readings) for (double b : a) if (b > whiteLineSignalThreshold) foundLine = true;

        if (foundLine) {
            wheels.stop();
            s = Stage.FINDING_FIRST_BEACON;
        } else {
            //ratio of y position to x position could be used to see if the robot is translating properly
            wheels.drive(0.2 * (isRed ? 1 : -1), 0.5, Utils.angleDifference(initialHeading, sensors.getHeading()) / Math.PI, false);
        }
    }


    private void findFirstBeacon() {

    }

    private void setUpShooter() {
        sensors.foldIMU();
        intake.releaseIMU();
        Utils.sleep(1500);
        intake.stop();
    }

    public void stop() {
        wheels.stop();
        intake.stop();
        shooter.stop();
    }

    private enum Stage {
        SHOOTING, FINDING_FIRST_WHITE_LINE, FINDING_FIRST_BEACON, PUSHING_BUTTON, FINDING_SECOND_WHITE_LINE, FINDING_SECOND_BEACON, KNOCKING_BALL, PARKING
    }

    public boolean isActive() { //returns true until all stages have been completed
        return isActive;
    }

}
