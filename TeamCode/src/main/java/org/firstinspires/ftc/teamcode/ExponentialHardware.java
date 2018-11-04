package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

public abstract class ExponentialHardware extends LinearOpMode {

    DcMotor lmotor0;
    DcMotor lmotor1;
    DcMotor rmotor0;
    DcMotor rmotor1;
    DcMotor hingeMotor;
    GyroSensor gyro;


    @Override
    public void runOpMode() throws InterruptedException {
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        hingeMotor = hardwareMap.dcMotor.get("hingeMotor");
        gyro = hardwareMap.gyroSensor.get("gryoSensor");
        while (gyro.isCalibrating())

        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);

        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hingeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }
}
