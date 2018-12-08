package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //hanging:
        // initAutoMotors(); //keep motors running for hang
        //add code to drop down
        initializeIMU();
        initVision();
        while(!isStarted()){
            moveHingeTo(45);
        }
        waitForStart();
        dropDown();
        hitGold();
        craterAutoMoveToCrater();
    }
}
