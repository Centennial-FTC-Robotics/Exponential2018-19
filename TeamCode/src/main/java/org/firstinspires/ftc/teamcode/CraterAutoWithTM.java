package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CraterAutoWithTM", group="TeleOp")

public class CraterAutoWithTM extends ExponentialFunctions {

    private float autoMoveSpeed = 0.4f;
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

        moveModified(-10, autoMoveSpeed);

        //team marker
        turnRelative(90);
        moveModified(-51, autoMoveSpeed);
        turnRelative(45);
        moveModified(-24, autoMoveSpeed);
        tmServo.setPosition(0);
        moveModified(24, autoMoveSpeed);
        turnRelative(-45);
        moveModified(51, autoMoveSpeed);
        turnRelative(-90);

        //sample
        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnRelative(40);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-50);
        }
        moveModified(-20, autoMoveSpeed);

        //park
        moveHingeTo(45);
        moveSlidesTo(slidesMax - 1000, 0.8f);
        if (goldPos.equals("LEFT")) {
            turnRelative(-40);
            moveModified(-6, autoMoveSpeed);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(50);
            moveModified(-6, autoMoveSpeed);
        }
        else {
            moveModified(-3, autoMoveSpeed);
        }
    }
}
