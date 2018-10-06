package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class ExponentialHardware extends OpMode {

    DcMotor lmotor0;
    DcMotor lmotor1;
    DcMotor rmotor0;
    DcMotor rmotor1;

    public void init() {

        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        lmotor0.setDirection(DcMotor.Direction.REVERSE);
        lmotor1.setDirection(DcMotor.Direction.REVERSE);
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
