package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.Hardware;

/**
 * Created by benorgera on 3/31/17.
 */

@TeleOp(name = "Test2", group = "4102")

public class Test2 extends LinearOpMode {

    public void runOpMode() {
        Hardware.init(hardwareMap, this, true,telemetry);
        Hardware.getIntake();

        waitForStart();

        Hardware.print("retract");
        Hardware.getButtonPusher();

        Hardware.sleep(3000);

        Hardware.print("semiextend");
        Hardware.clean();
        Hardware.init(hardwareMap, this, false, telemetry);
        Hardware.getButtonPusher();

        Hardware.sleep(3000);

        Hardware.print("push left");
        Hardware.clean();
        Hardware.init(hardwareMap, this, true, telemetry);
        Hardware.getButtonPusher();

        Hardware.sleep(3000);

        Hardware.getButtonPusher().push(true);

        Hardware.sleep(3000);
        Hardware.print("push right");
        Hardware.getButtonPusher().push(false);

        Hardware.sleep(3000);

        Hardware.print("intake");
        Hardware.getIntake().startIntaking(true);

        Hardware.sleep(3000);
        Hardware.getIntake().stopIntaking();

        Hardware.print("flaps");
        Hardware.getIntake().setFlaps(true);

        Hardware.sleep(3000);

        Hardware.getIntake().setFlaps(false);
    }

}
