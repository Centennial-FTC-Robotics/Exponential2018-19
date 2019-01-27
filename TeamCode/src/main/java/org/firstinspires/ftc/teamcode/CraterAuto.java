package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize sensors and position
        super.runOpMode();
        initializeIMU();
        initVision();
        while(!isStarted()){
            moveHingeTo(35);
            moveSlidesEncoderAbsolute(5, 0.5f);
        }
        waitForStart();

         //come off of lander
        moveHingeTo(10);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveIntakeArm(0.5f);
        moveSlidesEncoderAbsolute(4600, 0.8f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};

        //retract slides
        move(-5, 0.5f);
        moveSlidesEncoderAbsolute(50, 0.8f);
        move(3, 0.3f);

        //identify+hit gold and move into crater
        hitGoldCrater();
    }
}
