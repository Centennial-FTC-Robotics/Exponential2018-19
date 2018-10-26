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
            lmotor0.setPower(Range.clip(Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y-Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x,-1,1));
            lmotor1.setPower(Range.clip(Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y-Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x,-1,1));
            rmotor0.setPower(Range.clip(Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y+Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x, -1,1));
            rmotor1.setPower(Range.clip(Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y+Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x,-1,1));

            int position = hingeMotor.getCurrentPosition();
            float stickPos = Range.clip(gamepad1.right_stick_y, -1, 1);

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
