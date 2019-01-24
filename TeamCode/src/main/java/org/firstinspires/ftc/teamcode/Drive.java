package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class Drive extends ExponentialFunctions {

    public static float scale = 1;
    public static float fastScale = 1;
    public static float slowScale = (float) 0.2;
    public double currentIntakePower = 0;

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

            //move slides with joystick
            float slideSpeed = Range.clip(gamepad2.left_stick_y, -1, 1);
            moveSlides(slideSpeed);

            //shift
            if (gamepad2.x) {
                shiftTo(stronk);
            }
            if (gamepad2.y) {
                shiftTo(speed);
            }

            if (gamepad2.left_bumper) {
                //moveHingeTo(0);
            }
            else {
                //move hinge with joystick
                int hingePos = lHingeMotor.getCurrentPosition();
                float hingeSpeed = Range.clip(gamepad2.right_stick_y, -1, 1);
                moveHinge(hingePos, hingeSpeed);
            }

            //slow mode
            if (gamepad1.right_bumper) {
                scale = (float) slowScale;
            }

            //fast mode
            if (gamepad1.left_bumper) {
                scale = (float) fastScale;
            }

            if (gamepad2.dpad_up) {

                //currentIntakePower -= 1;
                moveIntakeArm(Range.clip(1, 0, 1));

            }

            else if (gamepad2.dpad_down) {

                //currentIntakePower += 1;
                moveIntakeArm(Range.clip(0, 0, 1));

            }
            else{
                moveIntakeArm(Range.clip(.5, 0, 1));

            }

            if (gamepad2.left_bumper) {
                moveIntake(Range.clip(-1, -1, 1));
            } else if(gamepad2.right_bumper){
                moveIntake(Range.clip(1, -1, 1));

                //moveIntake(Range.clip(currentIntakePower, -1, 1));
            }
            //moveHinge(getHingeTargetPos(), 0.1f);

            telemetry.addData("left Slide motor encoder: ", lSlideMotor.getCurrentPosition());
            telemetry.addData("right Slide motor encoder: ", rSlideMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
