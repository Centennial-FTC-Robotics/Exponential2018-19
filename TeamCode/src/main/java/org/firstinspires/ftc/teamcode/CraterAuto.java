package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
            moveSlidesTo(5, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(10);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveIntakeArm(0.5f);
        moveSlidesTo(4600, 0.8f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};

        //retract slides
        move(-5, 0.5f);
        moveSlidesTo(50, 0.8f);
        move(3, 0.3f);


        //----------------UNTESTED----------------//
        // move to depot
        float tempTurnSpeed = 0.4f;
        float tempMoveSpeed = 0.5f;
        turnRelative(45, tempTurnSpeed);
        move(-60, tempMoveSpeed);
        turnRelative(90, tempTurnSpeed);
        move(-72, tempMoveSpeed);

        tmServo.setPosition(0);

        // move back to lander
        move(72, tempMoveSpeed);
        turnRelative(-90, tempTurnSpeed);
        move(60, tempMoveSpeed);
        turnRelative(135, tempTurnSpeed);

        intakeGold();
        //add more if slides don't reach crater
    }
}