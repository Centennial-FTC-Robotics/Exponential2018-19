package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TeleOp")
//0 is front, 1 is back
//l means left, r means right
public class Drive extends ExponentialFunctions {

    public static float scale = 1;
    public static float fastScale = 1;
    public static float slowScale = (float) 0.5;
    public double currentIntakePower = 0;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        shifterServo.setPosition(speed);
        waitForStart();
        lmotor0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lmotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rmotor0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rmotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

            //move slides with joystick
            float slideSpeed = Range.clip(gamepad2.left_stick_y, -1, 1);
            moveSlidesUnlimited(slideSpeed);

            float hingeSpeed = Range.clip(gamepad2.right_stick_y, -0.5f, 0.5f);
            moveHinge(hingeSpeed);

            // Speed Scaling
            //slow mode
            if (gamepad1.right_bumper) {
                scale = (float) fastScale;
            }

            //fast mode
            if (gamepad1.left_bumper) {
                scale = (float) slowScale;
            }

            //shifting mechanism
            if (gamepad2.x) {
                shiftTo(stronk);
            }

            if (gamepad2.y) {
                shiftTo(speed);
            }

            //Intake Arms
            if (gamepad2.dpad_up) {
                moveIntakeArm(1);

            } else if (gamepad2.dpad_down) {
                moveIntakeArm(0);
            }
            else{
                moveIntakeArm(.45);
            }

            //Intake
            if (gamepad2.left_bumper) {
                moveIntake(-1);
            } else if(gamepad2.right_bumper){
                moveIntake(1);
            } else {
                moveIntake(0);
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
