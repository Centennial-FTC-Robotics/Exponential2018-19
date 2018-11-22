package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto1", group="TeleOp")

public class Autonomous extends ExponentialMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initAutoMotors();
        initVision();
        waitForStart();
        // write code to get down from hanging
        shift(); //switch to speed
        //while () {}
        // finding and displacing mineral

        String goldPos = autoFindGold();

        //team marker?
        //move(50); //yeet forward into crater


    }
}
