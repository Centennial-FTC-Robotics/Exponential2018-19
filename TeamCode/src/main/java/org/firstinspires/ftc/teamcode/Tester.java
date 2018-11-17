package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Tester", group="Autonomous")

public class Tester extends ExponentialMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        //move(12);
        turn(90,1);
    }
}
