package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class ExponentialMethods extends ExponentialHardware {

    private final DcMotor[] leftDriveMotors = {lmotor0, lmotor1};
    private final DcMotor[] rightDriveMotors = {rmotor0, rmotor1};
    private final DcMotor[] driveMotors = {lmotor0, lmotor1, rmotor0, rmotor1};

    private final int driveTicksPerRev = 560;
    private final int driveSprocket = 24;
    private final int wheelSprocket = 22;
    private final int wheelDiameterIn = 4;

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

    public void waitForMotors() {
        while (motorsBusy()) {
        }
    }

    public boolean motorsBusy() {
        boolean busy = false;
        for (DcMotor motor : driveMotors) {
            if (motor.isBusy()) {
                busy = true;
            }
        }
        return busy;
    }
    public void move(float distance) {
        //converting from linear distance -> wheel rotations ->
        // motor rotations -> encoder counts, then round
        float wheelRotations = (float)(distance / (wheelDiameterIn * Math.PI));
        float motorRotations = (22/24) * (wheelRotations);
        float encoderCounts = 560 * motorRotations;
        int position = Math.round(encoderCounts);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(position);
        }
        waitForMotors();
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
