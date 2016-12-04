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

    private boolean startOfTranslate = true;

    private boolean isActive = true;

    private final double odsThreshold = 0.04;

    private final double whiteLineSignalThreshold = 7; //the minimum color sensor reading required to signify finding the white line


    public AutonomousImplementation(boolean isRed) {
        this.wheels = Hardware.getWheels();
        this.sensors = Hardware.getSensors(wheels);
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
                findWhiteLine();
                break;
            case FINDING_FIRST_BEACON:
                findBeacon();
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
        intake.run(0.2);
        Utils.sleep(1500);

        //stop the intake and shooter
        intake.stop();
        shooter.stop();

        sensors.centerIMU();
        Utils.sleep(1500); //wait for things to steady before taking readings
        sensors.initImu();

        s = Stage.FINDING_FIRST_WHITE_LINE;
    }

    private void findWhiteLine() {
        double[][] readings = sensors.getLineData();
        boolean foundLine = false;

        for (double[] a : readings) for (double b : a) if (b > whiteLineSignalThreshold) foundLine = true;

        if (foundLine) {
            wheels.softStop();
            if (!isRed) sensors.turnAround(); //turn around if we're blue (one way or another)
            startOfTranslate = true;
            s = Stage.FINDING_FIRST_BEACON;
        } else {
            sensors.compensatedTranslate(Math.PI / 4 + (isRed ? Math.PI : 0), startOfTranslate, false);
        }

        startOfTranslate = false;
    }


    private void findBeacon() {

        if (sensors.getOpticalDistance() > odsThreshold) { //we've found the beacon
            s = Stage.PUSHING_BUTTON;
            wheels.stop();
            return;
        }

        sensors.followLine();
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