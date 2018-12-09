package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SingleDriverTeleOp", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class MatthewDrive extends ExponentialFunctions {

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

            //move hinge with dpad
            int hingePos = hingeMotor.getCurrentPosition();
            if (gamepad1.dpad_up) {
                moveHinge(hingePos, 0.5f);
            }
            else if (gamepad1.dpad_down) {
                moveHinge(hingePos, -0.5f);
            }
            else{
                moveHinge(hingePos, 0);
            }

            //move slides with joystick
            if (gamepad1.left_trigger != 0) {
                moveSlides(gamepad1.left_trigger);
            }
            else {
                moveSlides(-gamepad1.right_trigger);
            }

            //shift
            if (gamepad1.a) {
                shiftTo(stronk);
            }
//            if (gamepad1.b) {
//                shiftTo(speed);
//            }

            if (gamepad1.y) {
                moveHingeTo(0);
            }

            //slow mode
            if (gamepad1.left_bumper) {
                scale = (float) slowScale;
            }

            //fast mode
            if (gamepad1.right_bumper) {
                scale = (float) fastScale;
            }

            //moveHinge(getHingeTargetPos(), 0.1f);

            /*telemetry.addData("left Slide motor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("right Slide motor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.update();*/
            idle();
        }
    }
}
