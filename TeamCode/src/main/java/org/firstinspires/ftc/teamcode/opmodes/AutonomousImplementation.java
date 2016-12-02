package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.components.Hardware;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Integrator;
import org.firstinspires.ftc.teamcode.components.Sensors;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Utils;
import org.firstinspires.ftc.teamcode.components.Wheels;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Created by benorgera on 11/24/16.
 */

public class AutonomousImplementation {

    private Wheels wheels;
    private Sensors sensors;
    private Shooter shooter;
    private Intake intake;
    private Stage s = Stage.SHOOTING;

    private final boolean isRed;

    private boolean isActive = true;

    private final double k = 1;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line


    //color sensor representations
    private final int[] frontLeft = {0, 0};
    private final int[] frontRight = {0, 1};
    private final int[] backLeft = {1, 0};
    private final int[] backRight = {1, 1};


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

        sensors.centerIMU();
        Utils.sleep(1500); //wait for things to steady before taking readings
        sensors.initImu();

        sensors.resetHeading();
        s = Stage.FINDING_FIRST_WHITE_LINE;

    }

    private void findFirstWhiteLine() {
        double[][] readings = sensors.getLineData();
        boolean foundLine = false;

        for (double[] a : readings) for (double b : a) if (b > whiteLineSignalThreshold) foundLine = true;

        if (foundLine) {
            wheels.stop();
            if (!isRed) rotate(Math.random() > 0.5 ? -Math.PI : Math.PI); //turn around if we're blue (one way or another)
            s = Stage.FINDING_FIRST_BEACON;
        } else {
            translate(Math.PI / 4 + (isRed ? Math.PI : 0));
        }
    }


    private void findFirstBeacon() {


//        if (sensors.get)

        double[][] readings = sensors.getLineData(); //the array stores the readings on the button pusher side as the front

        ArrayList<Integer[]> maxReadingIndexes = Utils.findTwoMaxIndexes(readings);

//        if (maxReadingIndexes.contains(frontLeft)) {
//            if (maxReadingIndexes.contains(frontRight)) {
//                translate(Math.PI);
//            } else if (maxReadingIndexes.contains())
//        }

        if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(frontRight)) {

        } else if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(backRight)) {

        } else if (maxReadingIndexes.contains(frontLeft) && maxReadingIndexes.contains(backLeft)) {

        } else if (maxReadingIndexes.contains(frontRight) && maxReadingIndexes.contains(backLeft)) {

        } else if (maxReadingIndexes.contains(frontRight) && maxReadingIndexes.contains(backRight)) {

        } else if (maxReadingIndexes.contains(backLeft) && maxReadingIndexes.contains(backRight)) {

        }

    }

    public void stop() {
        wheels.stop();
        intake.stop();
        shooter.stop();
    }

    private enum Stage {
        SHOOTING, FINDING_FIRST_WHITE_LINE, FINDING_FIRST_BEACON, PUSHING_BUTTON, FINDING_SECOND_WHITE_LINE, FINDING_SECOND_BEACON, KNOCKING_BALL, PARKING
    }

    private void translate(double thetaDesired) { //translate the robot at desired angle [0, 2π], and compensate for unwanted rotation/drift our orientation and estimated position
        double ngVel = sensors.getHeading() / Math.PI, //compensate for rotation by accounting for change in heading
            thetaActual = Utils.atan3(sensors.getPosition().y, sensors.getPosition().x), //the direction of displacement thus far
                thetaDif = thetaDesired - thetaActual, //the difference between desired and actual displacement direction
                thetaNew = thetaDesired + thetaDif * k; //compensate for drift by accounting for the difference in desired and actual direction

        wheels.drive(Math.cos(thetaNew), Math.sin(thetaNew), ngVel, false);
    }

    private void rotate(double theta) { //rotate [-π, π]
        sensors.resetHeading();

        while (Math.abs(sensors.getHeading()) < Math.abs(theta))
            wheels.drive(0, 0, theta > 0 ? -0.3 : 0.3, false);

        wheels.stop();
    }

    public boolean isActive() { //returns true until all stages have been completed
        return isActive;
    }

}