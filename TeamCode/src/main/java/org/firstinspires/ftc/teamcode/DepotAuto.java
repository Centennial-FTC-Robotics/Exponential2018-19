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
        waitForStart();
        //shift(); //switch to speed
        ElapsedTime timer = new ElapsedTime();

        String goldPos = "bad";
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: " , timer.seconds());
            telemetry.update();
            goldPos = autoFindGold();
        }
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {
            turnRelative(27, 0.1);
            move(-37,0.2f);
            turnRelative(-54,0.1);
            move(-37,0.2f);
            turnRelative(27, 0.1);
        }
        else if (goldPos.equals("Center")) {
            move(-40, 0.2f);
        }
        else if (goldPos.equals("Right")) {
            turnRelative(-27, 0.1);
            move(-37,0.2f);
            turnRelative(54,0.1);
            move(-37,0.2f);
            turnRelative(-27, 0.1);
        }
        //team marker?
        //move(50); //yeet forward into crater

        turnRelative(-135, 0.1);
        move((-10 * 12), 0.2f);
    }
}
