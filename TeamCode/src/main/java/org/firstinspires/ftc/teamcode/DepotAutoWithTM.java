package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DepotAutoWithTM", group="TeleOp")

public class DepotAutoWithTM extends ExponentialFunctions {

    float autoMoveSpeed = 0.4f;
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize sensors and position
        super.runOpMode();
        initVision();
        initializeIMU();
        tmServo.setPosition(tmServoStart);
        while(!isStarted()){
            moveHingeTo(50);
            moveSlidesTo(5, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(10);
        while ((lHingeMotor.isBusy() || rHingeMotor.isBusy()) && opModeIsActive()) {}
        moveIntakeArm(0.5f);
        moveSlidesTo(4600, 0.8f);
        while ((lSlideMotor.isBusy() || rSlideMotor.isBusy()) && opModeIsActive()) {

            /*telemetry.addData("lSlideMotor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("rSlideMotor encoder: ", rSlideMotor.getCurrentPosition());
            telemet\ry.update();*/
        }

        //initializeIMU();
        resetOrientation();
        moveModified(-12, 0.4f);

        //sample
        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnRelative(40);
            moveModified(-22, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
            turnRelative(50);

        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-40);
            moveModified(-22, autoMoveSpeed);
            moveModified(18, autoMoveSpeed);
            turnRelative(130);
        }
        else {
            moveModified(-19, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
            turnRelative(90);
        }

        //team marker

        //diagonal move
        if (goldPos.equals("RIGHT")) {
            moveModified(-20, autoMoveSpeed);
        }
        else {
            moveModified(-15, autoMoveSpeed);
        }

        turnRelative(-45);

        //move towards wall
        if (goldPos.equals("LEFT")) {
            moveModified(-18, autoMoveSpeed);
        }
        else if (goldPos.equals("CENTER")) {
            moveModified(-24, autoMoveSpeed);
        }
        else if (goldPos.equals("RIGHT")) {
            moveModified(-21, autoMoveSpeed);
        }

        //turn towards depot
        if (goldPos.equals("LEFT")) {
            turnRelative(-89);
        }
        else if (goldPos.equals("CENTER")) {
            //turnRelative();
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-87);
        }

        //move to depot
        moveModified(-28, autoMoveSpeed);
        tmServo.setPosition(0);

        //move to crater
        moveModified(59, autoMoveSpeed);
    }
}
