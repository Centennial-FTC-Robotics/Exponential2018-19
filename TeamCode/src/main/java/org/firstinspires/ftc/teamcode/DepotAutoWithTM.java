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

        moveModified(-10, 0.4f);

        //sample
        String goldPos = findGold();
        moveModified(-2, 0.4f);
        if (goldPos.equals("LEFT")) {
            turnRelative(40);
            moveModified(-24, autoMoveSpeed);
            moveModified(22, autoMoveSpeed);
            turnRelative(-45);

        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-50);
            moveModified(-24, autoMoveSpeed);
            moveModified(22, autoMoveSpeed);
            turnRelative(45);
        }
        else {
            moveModified(-17, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
        }

        //team marker
        turnRelative(90);
        moveModified(-17, autoMoveSpeed);
        turnRelative(-45);
        moveModified(-24, autoMoveSpeed);
        turnRelative(-90);
        moveModified(-24, autoMoveSpeed);
        tmServo.setPosition(0);

        //park
        moveModified(50, autoMoveSpeed);
        moveHingeTo(45);
        moveSlidesTo(slidesMax - 1000, 0.8f);
    }
}
