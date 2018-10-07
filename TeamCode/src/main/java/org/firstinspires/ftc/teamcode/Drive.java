package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp", group="TeleOp")

public class Drive extends ExponentialMethods {

    public void start() {
        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        lmotor0.setPower(gamepad1.left_stick_y);
        lmotor0.setPower(gamepad1.left_stick_y);
        rmotor0.setPower(gamepad1.right_stick_y);
        rmotor0.setPower(gamepad1.right_stick_y);
    }
}
