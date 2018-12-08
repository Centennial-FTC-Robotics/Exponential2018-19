package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CraterAuto", group="TeleOp")

public class CraterAutoWithTM extends ExponentialMethods {

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

        dropDown(); //UNFINISHED METHOD

        //move to depot corner
        turnRelative(-45, turnSpeed);
        move(-48, moveSpeed);
        turnRelative(90, turnSpeed);
        move(-48, moveSpeed);

        //uhhhh wiggle and shake team marker off???
        moveHingeTo(20);
        for (int i = 0; i < 3; i++) {
            turnRelative(5, 2*turnSpeed);
            turnRelative(-5, 2*turnSpeed);
        }

        //move back to in front of crater
        move(48, moveSpeed);
        turnRelative(-90, turnSpeed);
        move(48, moveSpeed);
        turnRelative(45, turnSpeed);

        hitGold();
        craterAutoMoveToCrater();
    }
}
