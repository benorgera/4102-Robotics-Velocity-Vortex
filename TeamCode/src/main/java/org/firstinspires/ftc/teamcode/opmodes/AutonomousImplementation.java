package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    private Stage s = Stage.SHOOTING;

    public AutonomousImplementation(boolean isRed) {
        this.wheels = Hardware.getWheels();
        this.sensors = Hardware.getSensors();
        this.shooter = Hardware.getShooter();
        this.intake = Hardware.getIntake();
    }


    public void loop() {
        switch (s) {
            case SHOOTING:
                shoot();
                break;
            case FINDING_FIRST_BEACON:
                break;
            case PUSHING_BUTTON:
                break;
            case PARKING:
                break;
        }
    }

    private void shoot() {
        shooter.run(1); //get the shooters up to speed
        sensors.foldGyro(); //move the gyro out of the way of the intake

        Utils.sleep(1500);
        intake.run();
        Utils.sleep(1500);
        intake.stop();
        shooter.stop();

        sensors.centerGyro();

        s = Stage.FINDING_FIRST_BEACON;
    }

    public void stop() {
        wheels.stop();
        intake.stop();
        shooter.stop();
    }

    private enum Stage {
        SHOOTING, FINDING_FIRST_BEACON, PUSHING_BUTTON, FINDING_SECOND_BEACON, PARKING
    }

}
