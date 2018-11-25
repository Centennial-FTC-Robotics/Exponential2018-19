package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class Drive extends ExponentialMethods {

    public static float scale = 1;
    public static float fastScale = 1;
    public static float slowScale = (float) 0.2;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()){

            //drive with joysticks
            float moveSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y;
            float turnSpeed = Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            float leftSpeed;
            float rightSpeed;
            if (moveSpeed != 0) {
                leftSpeed = moveSpeed + (turnSpeed * moveSpeed);
                rightSpeed = moveSpeed - (turnSpeed * moveSpeed);
            }
            else {
                leftSpeed = -turnSpeed;
                rightSpeed = turnSpeed;
            }
            runDriveMotors(scale * leftSpeed, scale * rightSpeed);

            //move hinge with joystick
            int hingePos = hingeMotor.getCurrentPosition();
            float hingeSpeed = Range.clip(gamepad2.right_stick_y, -1, 1);
            moveHinge(hingePos, hingeSpeed);

            //move slides with joystick
            float slideSpeed = Range.clip(gamepad2.left_stick_y, -1, 1);
            moveSlides(slideSpeed);

            //shift
            if (gamepad2.a) {
                shiftTo(stronk);
            }

            if (gamepad2.b) {
                shiftTo(speed);
            }

            //slow mode
            if (gamepad1.left_bumper) {
                scale = (float) slowScale;
            }

            //fast mode
            if (gamepad1.right_bumper) {
                scale = (float) fastScale;
            }

            telemetry.addData("left Slide motor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("right Slide motor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
