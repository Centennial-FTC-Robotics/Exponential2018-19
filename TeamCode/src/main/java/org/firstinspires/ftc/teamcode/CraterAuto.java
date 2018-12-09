package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVision();
        waitForStart();

         //come off of lander
        dropDown();
        initializeIMU();

        //move so centered and facing crater
        move(9, moveSpeed);
        turnRelative(-90, turnSpeed);

        //identify+hit gold and move into crater
        hitGold();
    }
}
