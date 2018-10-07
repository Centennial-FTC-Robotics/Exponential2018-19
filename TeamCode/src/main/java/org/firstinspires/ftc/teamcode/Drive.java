package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp", group="TeleOp")

public class Drive extends ExponentialMethods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while(opModeIsActive()){
            lmotor0.setPower(gamepad1.left_stick_y);
            lmotor1.setPower(gamepad1.left_stick_y);
            rmotor0.setPower(gamepad1.right_stick_y);
            rmotor1.setPower(gamepad1.right_stick_y);
            idle();
        }
    }
}
