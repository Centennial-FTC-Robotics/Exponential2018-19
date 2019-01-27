package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CraterAutoWithTM", group="TeleOp")

public class CraterAutoWithTM extends ExponentialFunctions {

    @Override

    public void runOpMode() throws InterruptedException {

        // initialize sensors and position
        super.runOpMode();
        initializeIMU();
        initVision();
        while(!isStarted()){
            moveHingeTo(10);
            moveSlidesEncoderAbsolute(20, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(0);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveSlidesEncoderAbsolute(4200, 0.8f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};

        //retract slides
        move(-5, 0.5f);
        moveSlidesEncoderAbsolute(50, 0.8f);
        move(3, 0.5f);

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

        hitGoldCrater();
    }
}
