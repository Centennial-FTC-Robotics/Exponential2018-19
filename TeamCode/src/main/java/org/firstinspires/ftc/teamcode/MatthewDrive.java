package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MatthewDrive", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class MatthewDrive extends ExponentialFunctions {

    private static float scale = 1;
    private static float fastScale = 1;
    private static float slowScale = (float) 0.3;
    private boolean mode = true;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        waitForStart();

        while(opModeIsActive()){

            //drive with joysticks
            float moveSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y;
            float turnSpeed = Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            float leftSpeed = -turnSpeed;
            float rightSpeed = turnSpeed;

            if (moveSpeed != 0) {
                leftSpeed = moveSpeed + (turnSpeed * moveSpeed);
                rightSpeed = moveSpeed - (turnSpeed * moveSpeed);
            }

            runDriveMotors(scale * leftSpeed, scale * rightSpeed);

            //move hinge with dpad

            if (gamepad1.dpad_up && gamepad1.y) {

                mode = !mode;
                while (gamepad1.dpad_up && gamepad1.y) {}
            }

            if (mode) {
                int hingePos = lHingeMotor.getCurrentPosition();
                if (gamepad1.dpad_up) {
                    moveHinge(-0.5f);
                }
                else if (gamepad1.dpad_down) {
                    moveHinge(0.5f);
                }
                else{
                    moveHinge(0);
                }

                //move slides with joystick
                if (gamepad1.left_trigger != 0) {
                    moveSlides(gamepad1.left_trigger);
                }
                else {
                    moveSlides(-gamepad1.right_trigger);
                }

                //shift
                if (gamepad1.x) {
                    shiftTo(stronk);
                }
                if (gamepad1.y) {
                    shiftTo(speed);
                }

                if (gamepad1.y) {
                    moveHingeTo(0);
                }

                //despacito mode
                if (gamepad1.left_bumper) {
                    scale = slowScale;
                }

                //fast mode
                if (gamepad1.right_bumper) {
                    scale = fastScale;
                }
            } else {
                // intake code

                moveIntakeArm(Range.clip(gamepad1.right_trigger, -1, 1));
                moveIntake(-gamepad1.left_stick_y);
            }

            //moveHinge(getHingeTargetPos(), 0.1f);

            telemetry.addData("left Slide motor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("right Slide motor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.addData("Hinge Encoder: ", lHingeMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
