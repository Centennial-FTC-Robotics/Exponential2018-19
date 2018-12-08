package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HangTest", group="TeleOp")
public class HangTest extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while (opModeIsActive()) {
            moveHingeTo(45);
        }
    }
}
