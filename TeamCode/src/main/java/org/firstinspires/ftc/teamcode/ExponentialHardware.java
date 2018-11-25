package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// hardware
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

// sensors & sensorSetup
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class ExponentialHardware extends LinearOpMode {

    DcMotor lmotor0;
    DcMotor lmotor1;
    DcMotor rmotor0;
    DcMotor rmotor1;
    DcMotor hingeMotor;
    DcMotor lSlideMotor;
    DcMotor rSlideMotor;
    Servo shifterServo;

    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,0,0,0,0);
    BNO055IMU imu;
    double initialHeading;
    double initialPitch;
    double initialRoll;

    // slide shifting
    public static final double stronk = 0.0;
    public static final double speed = 0.45;

    DcMotor[] driveMotors=new DcMotor[4];
    @Override
    public void runOpMode() throws InterruptedException {
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        hingeMotor = hardwareMap.dcMotor.get("hingeMotor");
        lSlideMotor = hardwareMap.dcMotor.get("lSlide");
        rSlideMotor = hardwareMap.dcMotor.get("rSlide");
        shifterServo = hardwareMap.servo.get("shifterServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);

        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hingeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shifterServo.setPosition(stronk);

        driveMotors[0] = lmotor0;
        driveMotors[1] = lmotor1;
        driveMotors[2] = rmotor0;
        driveMotors[3] = rmotor1;

    }
}
