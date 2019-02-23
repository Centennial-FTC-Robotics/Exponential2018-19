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

        //retract slides
        move(-5, 0.4f);
        moveSlidesTo(50, 1f);
        move(3, 0.4f);


        //----------------UNTESTED----------------//
        // move to depot
        float tempMoveSpeed = 0.6f;
        turnRelative(45);
        move(-42, tempMoveSpeed);
        turnAbsoluteModified(45);
        turnRelative(90);
        move(-66, tempMoveSpeed);
        turnAbsoluteModified(135);

        tmServo.setPosition(0);

        // move back to lander
        move(68, tempMoveSpeed);
        turnAbsoluteModified(135);
        turnRelative(-90);
        move(36, tempMoveSpeed);
        turnAbsoluteModified(45);
        turnRelative(-45);

        intakeGold();
        //add more if slides don't reach crater
    }
}
