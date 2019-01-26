package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DepotAuto", group="TeleOp")

public class DepotAuto extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVision();

        while(!isStarted()){
            moveHingeTo(10);
            moveSlidesEncoderAbsolute(20, 0.5f);
        }
        waitForStart();

        //come off of lander
        moveHingeTo(0);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveSlidesEncoderAbsolute(4200, 0.8f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};

        //retract slides
        move(-5, 0.5f);
        moveSlidesEncoderAbsolute(50, 0.8f);
        move(3, 0.5f);

        hitGoldDepot();
    }
}
