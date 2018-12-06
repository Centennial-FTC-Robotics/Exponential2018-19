package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CraterAuto", group="TeleOp")

public class CraterAuto extends ExponentialMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //hanging:
        // initAutoMotors(); //keep motors running for hang
        //moveHinge(0);
        //add code to drop down
        initializeIMU();
        initVision();
        while(!isStarted()){
            moveHingeTo(45);
        }
        waitForStart();
        //shift(); //switch to speed
        ElapsedTime timer = new ElapsedTime();

        int lookAngle = 10;
        int turnAngle = 27;
        double turnSpeed = 0.3;
        float moveSpeed = 0.4f;
        String goldPos = "bad";

        //move to corner
        turnRelative(-45, turnSpeed);
        //move()

        //turn right to look at 2 minerals
        turnRelative(-lookAngle, turnSpeed);

        //figure out gold position
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: " , timer.seconds());
            telemetry.update();
            goldPos = autoFindGold2();
        }
        closeTfod();

        //turn back to starting position
        turnRelative(lookAngle, turnSpeed);

        //default to left if can't detect anything rip
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        //turn and move to hit
        if (goldPos.equals("Left")) {
            turnRelative(turnAngle, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(-turnAngle, turnSpeed);
            move(-6, moveSpeed);
        }
        else if (goldPos.equals("Center")) {
            move(-40, moveSpeed);
        }
        else if (goldPos.equals("Right")) {
            turnRelative(-turnAngle, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(turnAngle, turnSpeed);
            move(-6, moveSpeed);
        }

        //team marker?
        //move(50); //yeet forward into crater
    }
}
