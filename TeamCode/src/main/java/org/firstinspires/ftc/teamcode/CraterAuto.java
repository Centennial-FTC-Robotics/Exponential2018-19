package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //hanging:
        // initAutoMotors(); //keep motors running for hang
        //add code to drop down
        initializeIMU();
        initVision();
        /*while(!isStarted()){
            moveHingeTo(45);
        }
        waitForStart();
        dropDown();*/
        hitGold();
    }
}
