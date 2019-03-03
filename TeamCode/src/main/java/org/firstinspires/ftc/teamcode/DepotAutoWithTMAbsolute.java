package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DepotAutoWithTMAbsolute", group="TeleOp")

public class DepotAutoWithTMAbsolute extends ExponentialFunctions {

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
        while (opModeIsActive() && (lHingeMotor.isBusy() || rHingeMotor.isBusy())) {}
        moveIntakeArm(0.5f);
        moveSlidesTo(4600, 0.8f);
        while (opModeIsActive() && (lHingeMotor.isBusy() || rHingeMotor.isBusy())) {

            /*telemetry.addData("lSlideMotor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("rSlideMotor encoder: ", rSlideMotor.getCurrentPosition());
            telemet\ry.update();*/
        }

        //initializeIMU();
        resetOrientation();
        moveModified(-12, 0.4f);

        //sample
        String goldPos = findGold();

        //hit and move back
        if (goldPos.equals("LEFT")) {
            turnAbsoluteModified(40);
            moveModified(-22, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
        }
        else if (goldPos.equals("RIGHT")) {
            turnAbsoluteModified(-40);
            moveModified(-22, autoMoveSpeed);
            moveModified(16, autoMoveSpeed);
        }
        else {
            moveModified(-19, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
        }
        turnAbsoluteModified(90);
        //team marker

        //diagonal move
        if (goldPos.equals("RIGHT")) {
            moveModified(-20, autoMoveSpeed);
        }
        else {
            moveModified(-15, autoMoveSpeed);
        }

        turnAbsoluteModified(45);

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

        turnAbsoluteModified(-45);

        //move to depot
        moveModified(-28, autoMoveSpeed);

        dropMarker();

        //move to crater
        moveModified(61, autoMoveSpeed);
    }
}
