package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DepotAuto", group="TeleOp")

public class DepotAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision();

        while(!isStarted()){
            moveHingeTo(10);
            moveSlidesTo(20, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(0);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveSlidesTo(4200, 0.8f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};

        //retract slides
        move(-5, 0.5f);
        moveSlidesTo(50, 0.8f);
        move(3, 0.5f);

        //----------------UNTESTED----------------//
        intakeGold();

        float tempTurnSpeed = 0.4f;
        float tempMoveSpeed = 0.6f;
        float tempSlidesSpeed = 0.5f;
        moveHingeTo(0);
        moveSlidesTo(slidesMin, tempSlidesSpeed);

        turnRelative(45);
        move(-60, tempMoveSpeed);
        turnRelative(90);
        move(-48, tempMoveSpeed);
        tmServo.setPosition(0);
        move(84, tempMoveSpeed);
        moveSlidesTo(slidesMax, tempSlidesSpeed);
        moveHingeTo(45);
    }
}
