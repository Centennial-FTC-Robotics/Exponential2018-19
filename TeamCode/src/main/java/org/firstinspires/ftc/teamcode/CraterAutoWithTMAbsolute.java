package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CraterAutoWithTMAbsolute", group="TeleOp")

public class CraterAutoWithTMAbsolute extends ExponentialFunctions {

    private float autoMoveSpeed = 0.4f;
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
        while (opModeIsActive() && (lSlideMotor.isBusy() || rSlideMotor.isBusy())) {

            telemetry.addData("lSlideMotor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("rSlideMotor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.update();
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
            moveModified(13, autoMoveSpeed);
        }
        else {
            moveModified(-19, autoMoveSpeed);
            moveModified(15, autoMoveSpeed);
        }
        turnAbsoluteModified(90);

        //diagonal
        if (goldPos.equals("RIGHT")) {
            moveModified(-53.5f, autoMoveSpeed);
        } else if (goldPos.equals("LEFT")) {
            moveModified(-49f, autoMoveSpeed);
        } else if (goldPos.equals("CENTER")){
            moveModified(-48f, autoMoveSpeed);
        }
        turnAbsoluteModified(134);

        //move to depot
        moveModified(-25, autoMoveSpeed);

        dropMarker();

        //move to crater
        moveModified(62, autoMoveSpeed);

        /*moveModified(-18, autoMoveSpeed);

        //team marker

        //diagonal
        turnAbsoluteModified(90);
        moveModified(-51, 0.3f);

        //towards depot
        turnAbsoluteModified(135);
        moveModified(-22, autoMoveSpeed);

        tmServo.setPosition(0);

        //away from depot
        moveModified(26, autoMoveSpeed);
        turnAbsoluteModified(90);

        //diagonal back to start
        moveModified(46, autoMoveSpeed);
        turnAbsoluteModified(0);

        //sample
        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnAbsoluteModified(40);
        }
        else if (goldPos.equals("RIGHT")) {
            turnAbsoluteModified(-40);
        }
        moveModified(-20, autoMoveSpeed);

        //park
        turnAbsoluteModified(0);
        moveModified(-6, autoMoveSpeed);*/
    }
}
