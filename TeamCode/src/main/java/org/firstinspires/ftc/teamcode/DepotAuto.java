package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DepotAuto", group="TeleOp")

public class DepotAuto extends ExponentialMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //initAutoMotors(); //keep motors running for hang
        //add code to drop down
        initVision();
        initializeIMU();
        moveHingeTo(45);
        waitForStart();
        //shift(); //switch to speed
        ElapsedTime timer = new ElapsedTime();

        double turnSpeed = 0.3;
        float moveSpeed = 0.4f;
        String goldPos = "bad";

        //turn right to look at 2 minerals
        turnRelative(-10, turnSpeed);

        //figure out gold position
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: " , timer.seconds());
            telemetry.update();
            goldPos = autoFindGold();
        }
        closeTfod();

        //turn back to starting position
        turnRelative(10, turnSpeed);

        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {
            turnRelative(27, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(-54,turnSpeed);
            move(-37, moveSpeed);
            turnRelative(27, turnSpeed);
        }
        else if (goldPos.equals("Center")) {
            move(-40, moveSpeed);
        }
        else if (goldPos.equals("Right")) {
            turnRelative(-27, turnSpeed);
            move(-37,moveSpeed);
            turnRelative(54,turnSpeed);
            move(-37,moveSpeed);
            turnRelative(-27, turnSpeed);
        }
        //team marker?
        //move(50); //yeet forward into crater

        turnRelative(-135, 0.1);
        move((-10 * 12), 0.2f);
    }
}
