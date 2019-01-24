package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// hardware
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

// sensors & sensorSetup
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class ExponentialHardware extends LinearOpMode {

    DcMotorEx lmotor0;
    DcMotorEx lmotor1;
    DcMotorEx rmotor0;
    DcMotorEx rmotor1;
    DcMotorEx lHingeMotor;
    DcMotorEx rHingeMotor;
    DcMotorEx lSlideMotor;
    DcMotorEx rSlideMotor;
    Servo shifterServo;
    Servo lIntakeArmServo;
    Servo rIntakeArmServo;
    RevBlinkinLedDriver LEDStrip;
    CRServo lIntakeServo;
    CRServo rIntakeServo;


    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,0,0,0,0);
    BNO055IMU imu;
    double initialHeading;
    double initialPitch;
    double initialRoll;

    // slide shifting
    public static final double stronk = 0.03;
    public static final double speed = 0.45;

    DcMotorEx[] driveMotors;
    Servo[] intakeServos;
    @Override
    public void runOpMode() throws InterruptedException {
        lmotor0 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lmotor0");
        lmotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lmotor1");
        rmotor0 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rmotor0");
        rmotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rmotor1");
        lHingeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lHinge");
        rHingeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rHinge");
        lSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lSlide");
        rSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rSlide");
        shifterServo = hardwareMap.servo.get("shifterServo");
        lIntakeArmServo = hardwareMap.servo.get("lInArm");
        rIntakeArmServo = hardwareMap.servo.get("rInArm");
        LEDStrip = hardwareMap.get(RevBlinkinLedDriver.class, "StripAddressable");
        lIntakeServo = hardwareMap.crservo.get("lIntake");
        rIntakeServo = hardwareMap.crservo.get("rIntake");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        rmotor0.setDirection(DcMotorEx.Direction.REVERSE);
        rmotor1.setDirection(DcMotorEx.Direction.REVERSE);
        rHingeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rIntakeArmServo.setDirection(Servo.Direction.REVERSE);
        rIntakeServo.setDirection(CRServo.Direction.REVERSE);

        lmotor0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lHingeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rHingeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lmotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lHingeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rHingeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shifterServo.setPosition(stronk);
        intakeServos = new Servo[] {lIntakeArmServo, rIntakeArmServo};

        driveMotors = new DcMotorEx[] {lmotor0, lmotor1, rmotor0, rmotor1};

    }
}
