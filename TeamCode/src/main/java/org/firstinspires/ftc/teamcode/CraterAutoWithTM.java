package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CraterAutoWithTM", group="TeleOp")

public class CraterAutoWithTM extends ExponentialFunctions {

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

        dropDown(); //UNFINISHED METHOD // not anymore

        //move to depot corner
        turnRelative(-45, turnSpeed);
        move(-48, moveSpeed);
        turnRelative(90, turnSpeed);
        move(-48, moveSpeed);

        //uhhhh wiggle and shake team marker off??? haha hyes
        ejectTeamMarker(); // wiggle wiggle

        //move back to in front of crater
        move(48, moveSpeed);
        turnRelative(-90, turnSpeed);
        move(48, moveSpeed);
        turnRelative(45, turnSpeed);

        hitGold();
        craterAutoMoveToCrater();
    }
}
