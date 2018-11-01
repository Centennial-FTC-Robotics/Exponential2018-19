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
        while(opModeIsActive()){

            float leftSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y-Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            float rightSpeed = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y+Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x;
            runDriveMotors(leftSpeed, rightSpeed);
            int position = hingeMotor.getCurrentPosition();
            float stickPos = Range.clip(gamepad2.right_stick_y, -1, 1);

            //if at 90 degrees, only move if decreasing angle
            if (position >= 2240) {
                if (stickPos < 0) {
                    hingeMotor.setPower(stickPos);
                }
                else {
                    hingeMotor.setPower(0);
                }
            }

            //if at 0 degrees, only move if increasing angle
            else if (position <= 0){
                if (stickPos > 0) {
                    hingeMotor.setPower(stickPos);
                }
                else {
                    hingeMotor.setPower(0);
                }
            }

            //if in between 0 and 90 degrees
            else {
                hingeMotor.setPower(stickPos);
            }
            idle();
        }
    }
}
