package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HangTest", group="TeleOp")
public class HangTest extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initVision();
        initializeIMU();

        shiftTo(stronk);

        //hanging
        while(!isStarted()) {
            moveHingeTo(45);
            telemetry.addData("current angle: ", getRotationinDimension('z'));
            telemetry.update();
        }
        //wait
        waitForStart();

        //come off of lander
        dropDown();

        turnAroundLeftAbsolute(80, turnSpeed);
        moveSlidesTo(lSlideMotor.getCurrentPosition() + 70, 0.2f);
        moveHingeTo(35);
        moveSlidesTo(50, 0.2f);
        turnAbsolute(0, turnSpeed);

        while(true) {
            telemetry.addData("angle", getRotationinDimension('z'));
            telemetry.update();
        }
    }
}
