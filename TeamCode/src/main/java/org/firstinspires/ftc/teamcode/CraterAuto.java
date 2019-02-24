package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize sensors and position
        super.runOpMode();
        initVision();
        tmServo.setPosition(tmServoStart);
        while(!isStarted()){
            moveHingeTo(35);
            moveSlidesTo(5, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(10);
        while ((lHingeMotor.isBusy() || rHingeMotor.isBusy()) && opModeIsActive()) {}
        moveIntakeArm(0.5f);
        moveSlidesTo(4600, 0.8f);
        while ((lSlideMotor.isBusy() || rSlideMotor.isBusy()) && opModeIsActive()) {

            telemetry.addData("lSlideMotor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("rSlideMotor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.update();
        }

        initializeIMU();

        move(-10, 0.4f);

        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnRelative(40);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-40);
        }

        //extend slides out
        move(-20, 0.3f);
        //moveSlidesTo(slidesMax - 1000, 0.5f);
        moveHingeTo(45);

        //turn back toward crater
        if (goldPos.equals("LEFT")) {
            turnRelative(-40);
            move(-6, 0.3f);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(40);
            move(-6, 0.3f);
        }
        else {
            move(-3, 0.3f);
        }

        //----------------UNTESTED----------------//
        // move to depot
        /*move(-5, 0.4f);
        float tempMoveSpeed = 0.35f;
        turnRelative(87);
        move(-46, tempMoveSpeed);
        turnRelative(45);
        move(-26, tempMoveSpeed);

        tmServo.setPosition(0);

        move(34, tempMoveSpeed);
        turnRelative(-45);
        move(46, tempMoveSpeed);
        turnRelative(-90);
        move(14, tempMoveSpeed);*/

//------------
        /*turnRelative(45);
        move(-42, tempMoveSpeed);
        //turnAbsoluteModified(45);
        turnRelative(90);
        move(-66, tempMoveSpeed);
        //turnAbsoluteModified(135);

        //tmServo.setPosition(0);

        // move back to lander
        move(68, tempMoveSpeed);
        //turnAbsoluteModified(135);
        turnRelative(-90);
        move(36, tempMoveSpeed);
        //turnAbsoluteModified(45);
        turnRelative(-45);*/


        //add more if slides don't reach crater
    }
}
