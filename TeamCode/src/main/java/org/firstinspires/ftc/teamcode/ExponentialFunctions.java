package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public abstract class ExponentialFunctions extends ExponentialHardware {

    // simple conversions
    private static final float mmPerInch = 25.4f;

    //motors
    private final DcMotor[] leftDriveMotors = {lmotor0, lmotor1};
    private final DcMotor[] rightDriveMotors = {rmotor0, rmotor1};

    // hinge
    private int hingeTargetPos;

    // motor movement
    public static final int driveTicksPerRev = 560;
    public static final int driveSprocket = 24;
    public static final int wheelSprocket = 22;
    public static final int wheelDiameterIn = 4;
    public static final double default_P = 0;
    public static final double default_I = 0;
    public static final double default_D = 0;
    public static final double default_F = 0;

    // slides
    public int slidesMax = 5000;
    public int slidesMin = -20; //20;???

    // turn
    public static final int RIGHT = 1;
    public static final int LEFT = -1;

    // autonomous variables
    ElapsedTime timer;
    int lookAngle = 10;
    int turnAngle = 30;
    double turnSpeed = 0.4f;
    float moveSpeed = 0.5f;
    float tmServoStart = 0.5f;

    //tensor flow and vuforia stuff
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    private VuforiaLocalizer vuforia; //Vuforia localization engine
    private TFObjectDetector tfod; //Tensor Flow Object Detection engine

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaTrackables targetsRoverRuckus;
    private VuforiaLocalizer.Parameters parameters;
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        hingeTargetPos = lHingeMotor.getCurrentPosition();
    }
    /* -------------- Initialization -------------- */

    private void initVuforia() {
        //create parameter object and pass it to create Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        //create parameter object and pass it to create Tensor Flow object detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            telemetry.addData("oof", "oof");
            telemetry.update();
        }
        updateOrientation();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;
    }

    public void initVision() {
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();

            if (tfod != null) {

                tfod.activate();
            }

        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        //navTargetInit();
        //wait for game to start
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
    }
    /* -------------- Status Methods -------------- */

    public boolean motorsBusy() {
        boolean busy = false;
        for (int i = 0; i < 4; i++) {
            if (driveMotors[i].isBusy()) {
                busy = true;
            }
        }
        return busy;
    }

    public int getHingeTargetPos() {

        return hingeTargetPos;
    }

    public int getCurrentHingePos() {

        return (int) ((lHingeMotor.getCurrentPosition() + rHingeMotor.getCurrentPosition()) / 2.0);
    }

    public double getRawZ() {
        updateOrientation();
        return orientation.firstAngle;
    }

    public double getRawY() {
        updateOrientation();
        return orientation.thirdAngle;
    }

    public double getRawX() {
        updateOrientation();
        return orientation.secondAngle;
    }

    public double getRotationinDimension(char dimension) {

        switch (Character.toUpperCase(dimension)) {
            case 'X':

                return AngleUnit.normalizeDegrees(getRawX() - initialPitch);
            case 'Y':

                return AngleUnit.normalizeDegrees(getRawY() - initialRoll);
            case 'Z':

                return AngleUnit.normalizeDegrees(getRawZ() - initialHeading);
        }

        return 0;
    }

    public int getHingeAngle() {

        return lHingeMotor.getCurrentPosition() * (2240 / 90);
    }

    public void resetMotorEncoder(DcMotor motor) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* -------------- Correction -------------- */

    public void setOrientation(Orientation newO) {

        orientation = newO;
    }
    /* -------------- Processing -------------- */

    public double getAngleDist(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }

    public static int getAngleDir(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }

    public static double standardPosAngle(Vector v) {

        v.collapse(2);

        Vector i = new Vector(new double[] {1, 0});
        Vector j = new Vector(new double[] {0, 1});

        double iAngle = v.angleBetween(i);

        if (v.angleBetween(j) > 90) {

            iAngle = 360 - iAngle;
        }

        return iAngle;
    }

    public static int convertInchToEncoder(float dist) {

        float wheelRotations = (float) (dist / (wheelDiameterIn * Math.PI));
        float motorRotations = (float) ((22.0 / 24.0) * (wheelRotations));
        float encoderCounts = 560 * motorRotations;
        int position = Math.round(encoderCounts);

        return position;
    }

    public static double convertEncoderToInch(int encoders) {

        float motorRotations = encoders / 560;
        double wheelRotations = motorRotations * (24.0 / 22.0);
        double distance = wheelRotations * (ExponentialFunctions.wheelDiameterIn * Math.PI);

        return distance;
    }
    /* -------------- Movement -------------- */

    //movement based on speeds

    public static boolean setPID(DcMotorEx motor, PIDFCoefficients coeff) {

        return setPID(motor, coeff.p, coeff.i, coeff.d, coeff.f);
    }

    public static boolean setPID(DcMotorEx motor, double P, double I, double D, double F) {

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(P, I, D, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        return ((pidModified.p == P) && (pidModified.i == I) && (pidModified.d == D) && (pidModified.f == F));
    }

    public void runRightMotors(float speed) {
        rmotor0.setPower(Range.clip(speed, -1, 1));
        rmotor1.setPower(Range.clip(speed, -1, 1));
    }

    public void runLeftMotors(float speed) {
        lmotor0.setPower(Range.clip(speed, -1, 1));
        lmotor1.setPower(Range.clip(speed, -1, 1));
    }

    public void runDriveMotors(float leftSpeed, float rightSpeed) {
        lmotor0.setPower(Range.clip(leftSpeed, -1, 1));
        lmotor1.setPower(Range.clip(leftSpeed, -1, 1));
        rmotor0.setPower(Range.clip(rightSpeed, -1, 1));
        rmotor1.setPower(Range.clip(rightSpeed, -1, 1));
    }

    public void moveSlides(float power) {
        double direction = -1;
        int currentPos = (lSlideMotor.getCurrentPosition() + rSlideMotor.getCurrentPosition()) / 2;

        if (currentPos <= slidesMax && currentPos >= slidesMin) {
            lSlideMotor.setPower(direction * Range.clip(power, -1, 1));
            rSlideMotor.setPower(direction * Range.clip(power, -1, 1));
        } else if (currentPos > slidesMax) {
            if (power > 0) {
                lSlideMotor.setPower(direction * Range.clip(power, -1, 1));
                rSlideMotor.setPower(direction * Range.clip(power, -1, 1));
            } else {
                slidesBrake();
            }
        } else if (currentPos < slidesMin) {
            if (power < 0) {
                lSlideMotor.setPower(direction * Range.clip(power, -1, 1));
                rSlideMotor.setPower(direction * Range.clip(power, -1, 1));
            } else {
                slidesBrake();
            }
        }
    }

    public void moveSlidesUnlimited(float power) {
        double direction = -1;
        lSlideMotor.setPower(direction * Range.clip(power, -1, 1));
        rSlideMotor.setPower(direction * Range.clip(power, -1, 1));
    }

    public void moveSlidesTo(int encoderPos, float speed) {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lSlideMotor.setTargetPosition(encoderPos);
        rSlideMotor.setTargetPosition(encoderPos);
        lSlideMotor.setPower(speed);
        rSlideMotor.setPower(speed);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlideMotor.setPower(0);
        rSlideMotor.setPower(0);
    }

    public void moveHinge(float hingeSpeed) {
        lHingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rHingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if at 90 degrees, only move if decreasing angle
        int hingePos = getCurrentHingePos();

        if (hingePos >= 2200) {
            if (hingeSpeed < 0) {
                lHingeMotor.setPower(hingeSpeed);
                rHingeMotor.setPower(hingeSpeed);
                hingeTargetPos = hingePos;
            } else {
                lHingeMotor.setPower(0);
                rHingeMotor.setPower(0);
            }
        }
        //if at 0 degrees, only move if increasing angle
        else if (hingePos <= 0) {
            if (hingeSpeed > 0) {
                lHingeMotor.setPower(hingeSpeed);
                rHingeMotor.setPower(hingeSpeed);
                hingeTargetPos = hingePos;
            } else {
                lHingeMotor.setPower(0);
                rHingeMotor.setPower(0);
            }
        }
        //if in between 0 and 90 degrees, move however
        else {
            lHingeMotor.setPower(hingeSpeed);
            rHingeMotor.setPower(hingeSpeed);
            hingeTargetPos = hingePos;
        }
    }

    /*public void moveSlidesInchRelative(double targetΔ, double speed) {
        moveSlidesInchAbsolute(getSlideExtendInch() + targetΔ, speed);
    }

    public void moveSlidesInchAbsolute(double targetInch, double speed) {
        int targetPos = (int) (targetInch * (1 / slideInchPerStrInch));
        if (shifterServo.getPosition() == stronk) {
            targetPos /= inchesPerEncoderStronk;
        } else if (shifterServo.getPosition() == speed) {
            targetPos /= inchesPerEncoderSpeed;
        }

        if (targetPos > 1400) {
            targetPos = 1400;
        }

        moveSlidesAbsolute(targetPos, speed);
    }*/

    /*public void moveSlidesRelative(int targetΔ, double speed) {
        int currentPos = (lSlideMotor.getCurrentPosition() + rSlideMotor.getCurrentPosition()) / 2;
        moveSlidesAbsolute(currentPos + targetΔ, speed);
    }*/

    /*public void moveSlidesAbsolute(int targetPos, double speed) {
        lSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int currentPos = (lSlideMotor.getCurrentPosition() + rSlideMotor.getCurrentPosition()) / 2;

        lSlideMotor.setTargetPosition(targetPos);
        rSlideMotor.setTargetPosition(targetPos);
        lSlideMotor.setPower(Range.clip(speed, -1, 1));
        rSlideMotor.setPower(Range.clip(speed, -1, 1));

        // recording encoders moved
        if (shifterServo.getPosition() == stronk) {

            encodersMovedStronk += (currentPos - targetPos);
        } else if (shifterServo.getPosition() == speed) {

            encodersMovedSpeed += (currentPos - targetPos);
        }

        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {
        }
        ;
        lSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesBrake();
    }*/

    public void moveHingeTo(float angle) {
        lHingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rHingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle = Range.clip(angle, 0, 90);
        int position = (int) (angle * (2240 / 90));

/*        if (angle > 25 && getHingeAngle() < angle) {
            moveSlidesInchAbsolute(1, 0.1);
        }
        if (angle < 25 && getHingeAngle() > angle) {
            moveSlidesInchAbsolute(1, 0.1);
        }*/

        lHingeMotor.setTargetPosition(position);
        rHingeMotor.setTargetPosition(position);
        lHingeMotor.setPower(0.5);
        rHingeMotor.setPower(0.5);
        hingeTargetPos = position;
        //hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(Vector v, float speed) {

        // current orientation
        double currentAngle = getRotationinDimension('Z');
        Vector orientationVector = new Vector(new double[] {Math.cos(currentAngle), Math.sin(currentAngle)});

        // absolute angle of the target vector in standard position
        double targetAngle = standardPosAngle(v);

        // angle and angle direction
        double moveAngle = orientationVector.angleBetween(v) * getAngleDir(targetAngle, currentAngle);

        telemetry.addData("moveAngle: ", moveAngle);
        telemetry.update();

        turnRelative(moveAngle);
        waitForMotors();

        move((float) (v.getMagnitude()), speed);
        waitForMotors();

        turnRelative(-moveAngle);
        telemetry.addData("currentAngle: ", orientationVector.angleBetween(v) * getAngleDir(targetAngle, currentAngle));
        telemetry.update();
        waitForMotors();
    }

    //currently in inches
    public void move(float distance, float speed) {
        //converting from linear distance -> wheel rotations ->
        // motor rotations -> encoder counts, then round
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForMotors();

        int position = convertInchToEncoder(distance);

        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotors[i].setTargetPosition(-position);
            driveMotors[i].setPower(speed);
        }
        waitForMotors();

        runDriveMotors(0, 0);
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnRelative(double targetΔ) {

        turnAbsoluteModified(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + targetΔ));
    }

    public void turnAbsoluteModified(double targetAngle) {
        double currentAngle = getRotationinDimension('Z');
        int direction;
        double turnRate = 0;
        double P = 1d / 140d;
        double minSpeed = 0;
        double maxSpeed = 0.2d;
        double tolerance = .5;

        double error = getAngleDist(targetAngle, currentAngle);
        while (opModeIsActive() && error > tolerance) {
            currentAngle = getRotationinDimension('Z');
            direction = getAngleDir(targetAngle, currentAngle);
            error = getAngleDist(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            runDriveMotors((float) -(turnRate * direction), (float) (turnRate * direction));
            /*telemetry.addData("error: ", error);
            telemetry.addData("currentAngle: ", getRotationinDimension('Z'));
            telemetry.update();*/
        }
        runDriveMotors(0, 0);
    }


    public void turnAbsolute(double targetAngle, double speed) {
        double currentAngle = getRotationinDimension('Z');
        int direction;
        if (targetAngle != currentAngle) {

            double angleDifference = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);

            double turnRate = (angleDifference * speed) / 90;

            while (opModeIsActive() && angleDifference > 1) {
                runDriveMotors((float) -(turnRate * direction), (float) (turnRate * direction));
                angleDifference = getAngleDist(targetAngle, getRotationinDimension('Z'));
                direction = getAngleDir(targetAngle, getRotationinDimension(('Z')));
                turnRate = (angleDifference * speed) / 90;
                telemetry.addData("angleDifference: ", angleDifference);
                telemetry.addData("currentAngle: ", getRotationinDimension('Z'));
                telemetry.update();
            }

            runDriveMotors(0, 0);
        }
    }

    public void turnAroundLeftAbsolute(double targetAngle, double speed) {
        double currentAngle = getRotationinDimension('Z');
        int direction;
        if (targetAngle != currentAngle) {

            double angleDifference = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);

            double turnRate = (angleDifference * speed) / 90;

            while (opModeIsActive() && angleDifference > 1) {

                runRightMotors((float) (turnRate * direction));
                angleDifference = getAngleDist(targetAngle, getRotationinDimension('Z'));
                direction = getAngleDir(targetAngle, getRotationinDimension(('Z')));
                turnRate = (angleDifference * speed) / 90;
                telemetry.addData("angleDifference: ", angleDifference);
                telemetry.addData("currentAngle: ", getRotationinDimension('Z'));
                telemetry.update();
            }

            runRightMotors(0);
        }
    }

    public void shiftTo(double mode) {

        shifterServo.setPosition(mode);
    }

    public void dropDown() {
        moveHingeTo(0);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveSlidesTo(2100, 0.2f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};
    }

    public void hitGold() {

        String goldPos = "bad";
        Vector[] mineralPositions = new Vector[] {
                new Vector(new double[] {-Math.sqrt(2), Math.sqrt(8)}),
                new Vector(new double[] {Math.sqrt(2), Math.sqrt(8)}),
                new Vector(new double[] {0, Math.sqrt(8)})
        };

        Vector j = new Vector(new double[] {0, 1});

        double[] angles = new double[] {mineralPositions[0].angleBetween(j), mineralPositions[1].angleBetween(j), mineralPositions[2].angleBetween(j)};

        //turn right to look at 2 minerals
        turnRelative(-lookAngle);

        //figure out gold position
        timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 3 && goldPos.equals("bad")) {
            telemetry.addData("Timer: ", timer.seconds());

            goldPos = autoFindGold();

            telemetry.addData("Gold: ", goldPos);
            telemetry.update();

        }

        Vector movement = new Vector(new double[] {0.1, 0.1});
        movement.scale(12);
        move(movement, 0.4f);

        if (goldPos.equals("bad")) {

            for (Vector v: mineralPositions) {

                v.sub(movement);
            }

            for (int a = 0; a < angles.length; a++) {

                angles[a] = mineralPositions[a].angleBetween(j);
            }

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 3 && goldPos.equals("bad")) {
                telemetry.addData("Timer: ", timer.seconds());

                goldPos = autoFindGold();

                telemetry.addData("Gold: ", goldPos);
                telemetry.update();

            }
        }

        closeTfod();

        //turn back to starting position
        turnRelative(lookAngle);

        //default to left if can't detect anything rip
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {

            turnRelative(angles[0]);
        } else if (goldPos.equals("Right")) {

            turnRelative(angles[1]);
        } else if (goldPos.equals("Center")) {

            turnRelative(angles[2]);
        }
    }

    public void moveIntakeArm(double newPos) {
        for (int IServo = 0; IServo < intakeServos.length; IServo++) {
            intakeServos[IServo].setPosition(newPos);
        }
    }

    public void moveIntake(double power) {

        lIntakeServo.setPower(power);
        rIntakeServo.setPower(power);

    }
    /* -------------- Procedure -------------- */

    public void waitForMotors() {
        while (opModeIsActive() && motorsBusy()) {
        }
    }

    public static void waitTime(int time) {
        try {

            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void slidesBrake() {
        lSlideMotor.setPower(0);
        rSlideMotor.setPower(0);
    }

    /* -------------- Computer Vision -------------- */

    //returns left, right, or center based on position of gold

    public String autoFindGold() {
        //outputs gold position from 2 sensed objects
        String goldPosition = "bad";

        if (opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                // goldPosition = evaluateRecognitions(updatedRecognitions);
                goldPosition = evaluateRecognitionsFOV(updatedRecognitions);
            }
        }

        //added:
        return goldPosition;
    }

    public String autoFindGold(int evaluator) {
        //outputs gold position from 2 sensed objects
        String goldPosition = "bad";

        if (opModeIsActive()) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                switch (evaluator) {
                    case 1:

                        goldPosition = evaluateRecognitions(updatedRecognitions);
                        break;
                    case 2:

                        goldPosition = evaluateRecognitionsFOV(updatedRecognitions);
                        break;
                }
            }
        }

        //added:
        return goldPosition;
    }

    public void closeTfod() {

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private String evaluateRecognitions(List<Recognition> recognitions) {

        String goldPosition = "bad";

        if (recognitions != null) {
            telemetry.addData("# Object Detected", recognitions.size());
            if (recognitions.size() == 2) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                //gets x positions for each mineral detected
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getBottom();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getBottom();
                    } else {
                        silverMineral2X = (int) recognition.getBottom();
                    }
                }

                //determines position of gold mineral
                if (goldMineralX == -1) {
                    goldPosition = "Left";
                } else if (goldMineralX > silverMineral1X) {
                    goldPosition = "Center";
                } else if (goldMineralX < silverMineral1X) {
                    goldPosition = "Right";
                }
            }

            telemetry.update();
        }

        return goldPosition;
    }

    private String evaluateRecognitionsFOV(List<Recognition> recognitions) {

        String goldPosition = "bad";

        if (recognitions != null) {
            telemetry.addData("# Object Detected", recognitions.size());
            if (recognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;

                //gets x positions for each mineral detected
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getBottom();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getBottom();
                    } else {
                        silverMineral2X = (int) recognition.getBottom();
                    }
                }

                //determines position of gold mineral
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        //added:
                        goldPosition = "Left";

                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        //added:
                        goldPosition = "Right";
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        //added:
                        goldPosition = "Center";
                    }
                }
            }
            telemetry.update();
        }

        return goldPosition;
    }

    public String identifySingleMineral() {
        String color = "SILVER";
        while (opModeIsActive() && timer.seconds() < 2) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                    Recognition mineral = null;
                    double highestConfidence = -1;
                    for (Recognition r : updatedRecognitions) {
                        if (r.getConfidence() > highestConfidence) {
                            mineral = r;
                        }
                    }
                    if (mineral.getLabel().equals("LABEL_GOLD_MINERAL")) {
                        color = "GOLD";
                    }
                    else {
                        color = "SILVER";
                    }
                }
            }
        }
        return color;
    }

    public String findGold() {
        String right;
        String center;
        String goldPos = "LEFT";

        //move();
        turnRelative(37);
        right = identifySingleMineral();
        turnRelative(-37);
        center = identifySingleMineral();

        if (right.equals("SILVER") && center.equals("SILVER")) {
            goldPos = "LEFT";
        }
        else if (right.equals("GOLD")) {
            goldPos = "CENTER";
        }
        else {
            goldPos = "RIGHT";
        }

        return goldPos;
    }

    public void intakeGold() {
        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnRelative(-37);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(37);
        }

        //extend slides out and intake mineral
        moveSlidesTo(slidesMax - 1000, 0.5f);
        moveIntakeArm(1);
        moveHingeTo(90);
        moveIntake(-1);
        moveSlidesTo(slidesMax, 0.5f);
    }

//    public void updateNavTargets() {
//
//        // check all the trackable target to see which one (if any) is visible.
//        targetVisible = false;
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                telemetry.addData("Visible Target", trackable.getName());
//                targetVisible = true;
//
//                // getUpdatedRobotLocation() will return null if no new information is available since
//                // the last time that call was made, or if the trackable is not currently visible.
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//                break;
//            }
//        }
//
//        // Provide feedback as to where the robot is located (if we know).
//        if (targetVisible) {
//            // express position (translation) of robot in inches.
//            VectorF translation = lastLocation.getTranslation();
//            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//            // express the rotation of the robot in degrees.
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//        } else {
//            telemetry.addData("Visible Target", "none");
//        }
//        telemetry.update();
//    }
}