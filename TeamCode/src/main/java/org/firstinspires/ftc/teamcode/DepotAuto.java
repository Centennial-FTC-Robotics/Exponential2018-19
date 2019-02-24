package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DepotAuto", group="TeleOp")

public class DepotAuto extends ExponentialFunctions {

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

        //----------------UNTESTED----------------//
        move(-10, 0.4f);
        float moveSpeed = 0.3f;
        String goldPos = findGold();
        move(-2, 0.4f);

        if (goldPos.equals("LEFT")) {
            turnRelative(45);
            move(-28, moveSpeed);
            turnRelative(90);
            move(-40, moveSpeed);

        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-45);
            move(-24, moveSpeed);
            move(22, moveSpeed);
            turnRelative(45);

            turnRelative(90);
            move(-48, moveSpeed);
            turnRelative(45);
            move(-3, moveSpeed);
            moveHingeTo(45);
        }
        else {
            move(-17, moveSpeed);
            move(15, moveSpeed);

            turnRelative(90);
            move(-48, moveSpeed);
            turnRelative(45);
            move(-3, moveSpeed);
            moveHingeTo(45);
        }
        //move (-3, 0.4f);



        /*Vector movement;
        Vector turn = new Vector(new double[] {-3 * Math.sqrt(2), Math.sqrt(2)});

        if (goldPos.equals("LEFT")) {

            movement = new Vector(new double[] {-Math.sqrt(2), 2 * Math.sqrt(2)});

        }
        else if (goldPos.equals("RIGHT")) {

            movement = new Vector(new double[] {Math.sqrt(2), 2 * Math.sqrt(2)});
        }
        else {

            movement = new Vector(new double[] {0, 2 * Math.sqrt(2)});
        }

        movement.scale(12);
        turn.scale(12);

        move(movement, 0.3f);
        turn.sub(movement);
        waitForMotors();

        movement.scale(0.5);
        movement = Vector.invert(movement);

        move(movement, 0.3f);
        turn.sub(movement);
        waitForMotors();

        move(turn, 0.3f);

        turnRelative(135);*/
    }
}
