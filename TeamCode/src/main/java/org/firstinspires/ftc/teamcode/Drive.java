package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class Drive extends ExponentialMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()){

            //drive with joysticks
            float leftSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y-Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            float rightSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y+Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            runDriveMotors(leftSpeed, rightSpeed);

            //move hinge with joystick
            int hingePos = hingeMotor.getCurrentPosition();
            float hingeSpeed = Range.clip(gamepad2.right_stick_y, -1, 1);
            moveHinge(hingePos, hingeSpeed);

            //move slides with joystick
            float slideSpeed = Range.clip(gamepad2.left_stick_y, -1, 1);
            moveSlides(slideSpeed);

            //shift
            if (gamepad2.a) {
                shift();
            }
            idle();
        }
    }
}
