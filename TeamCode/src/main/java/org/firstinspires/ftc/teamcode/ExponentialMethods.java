package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class ExponentialMethods extends ExponentialHardware {

    public void runDriveMotors(float leftSpeed, float rightSpeed) {
        lmotor0.setPower(Range.clip(leftSpeed, -1, 1));
        lmotor1.setPower(Range.clip(leftSpeed, -1, 1));
        rmotor0.setPower(Range.clip(rightSpeed, -1, 1));
        rmotor1.setPower(Range.clip(rightSpeed, -1, 1));
    }

    public void moveHingeTo(float angle) {
        hingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle = Range.clip(angle, 0, 90);
        int position = (int)(angle * (2240/90));
        hingeMotor.setTargetPosition(position);
        hingeMotor.setPower(1);
        hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
