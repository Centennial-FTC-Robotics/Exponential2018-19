package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto1", group="TeleOp")

public class Autonomous extends ExponentialMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        move(12);
    }
}
