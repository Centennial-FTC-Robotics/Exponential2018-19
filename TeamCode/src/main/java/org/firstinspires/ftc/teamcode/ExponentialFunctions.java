package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;   // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;

    //motors
    private final DcMotor[] leftDriveMotors = {lmotor0, lmotor1};
    private final DcMotor[] rightDriveMotors = {rmotor0, rmotor1};

    // hinge
    private int hingeTargetPos;

    // motor movement
    public final int driveTicksPerRev = 560;
    public final int driveSprocket = 24;
    public final int wheelSprocket = 22;
    public final int wheelDiameterIn = 4;

    // slides
    public int slidesMax = 4600;
    public int slidesMin = -20; //20;???
    private int encodersMovedStronk;
    private int encodersMovedSpeed;
    private double slideSpoolInnerDiameter = 0.8;
    public double inchesPerEncoderStronk = (Math.PI * slideSpoolInnerDiameter) / (28 * 10 * 3);
    public double inchesPerEncoderSpeed = (Math.PI * slideSpoolInnerDiameter) / (28 * 10);
    public double slideInchPerStrInch = 1.0; // replace w/ actual value

    // turn
    public static final int RIGHT = 1;
    public static final int LEFT = -1;

    // autonomous variables
    ElapsedTime timer;
    int lookAngle = 10;
    int turnAngle = 30;
    double turnSpeed = 0.4f;
    float moveSpeed = 0.5f;

    //tensor flow and vuforia stuff
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    private VuforiaLocalizer vuforia; //Vuforia localization engine
    private TFObjectDetector tfod; //Tensor Flow Object Detection engine
    private int cameraMonitorViewId;

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

        encodersMovedSpeed = 0;
        encodersMovedSpeed = 0;
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

    private void navTargetInit() {

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.useExtendedTracking = true;
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        targetsRoverRuckus.activate();
    }

    public void autoInit() {
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        //wait for game to start
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
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

    public void initAutoMotors() {

        moveSlides(1);
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

    public OpenGLMatrix getRotation() {
        updateOrientation();
        return orientation.getRotationMatrix();
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

    public double getSlideExtendInch() {

        double strInches = (encodersMovedSpeed * inchesPerEncoderSpeed) + (encodersMovedStronk * inchesPerEncoderStronk);

        return (strInches * slideInchPerStrInch);
    }

    public double getSlideMode() {

        return shifterServo.getPosition();
    }

    public void resetMotorEncoder(DcMotor motor) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* -------------- Correction -------------- */

    public void setSlideZero() {

        encodersMovedSpeed = 0;
        encodersMovedStronk = 0;
    }

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

    public int getAngleDir(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));
        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }
        return angleDir;
    }

    public int convertInchToEncoder(float dist) {

        float wheelRotations = (float) (dist / (wheelDiameterIn * Math.PI));
        float motorRotations = (float) ((22.0 / 24.0) * (wheelRotations));
        float encoderCounts = 560 * motorRotations;
        int position = Math.round(encoderCounts);

        return position;
    }

    public double convertEncoderToInch(int encoders) {

        float motorRotations = encoders / 560;
        double wheelRotations = motorRotations * (24.0 / 22.0);
        double distance = wheelRotations * (wheelDiameterIn * Math.PI);

        return distance;
    }
    /* -------------- Movement -------------- */

    //movement based on speeds

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

    public void turnRelative(double targetΔ, double speed) {

        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + targetΔ), speed);
    }

    public void turnAbsoluteModified(double targetAngle, double maxSpeed) {
        double currentAngle = getRotationinDimension('Z');
        int direction;
        double turnRate = 0;
        double P = 1d / 90d;
        double minSpeed = 0;
        int tolerance = 1;

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

        moveSlidesTo(0, 0.1f);
        shifterServo.setPosition(mode);
    }

    public void dropDown() {
        moveHingeTo(0);
        while (lHingeMotor.isBusy() || rHingeMotor.isBusy()) {};
        moveSlidesTo(2100, 0.2f);
        while (lSlideMotor.isBusy() || rSlideMotor.isBusy()) {};
    }

    public void intakeGold() {
        float turnSpeed = 0.5f;
        String goldPos = autoFindGold();
        if (goldPos.equals("Left")) {
            turnRelative(-37, turnSpeed);
        }
        else if (goldPos.equals("Right")) {
            turnRelative(37, turnSpeed);
        }

        //extend slides out and intake mineral
        moveSlidesTo(slidesMax - 1000, 0.5f);
        moveIntakeArm(1);
        moveHingeTo(90);
        moveIntake(-1);
        moveSlidesTo(slidesMax, 0.5f);
    }

    public void hitGoldCrater() {

        String goldPos = "bad";

        //turn right to look at 2 minerals
        turnRelative(-lookAngle, turnSpeed);

        //figure out gold position
        timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: ", timer.seconds());

            goldPos = autoFindGold();
            telemetry.addData("Gold: ", goldPos);
            telemetry.update();

        }

        closeTfod();

        //turn back to starting position
        turnRelative(lookAngle - 5, turnSpeed);

        //default to left if can't detect anything rip
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        //turn and move to hit
        if (goldPos.equals("Left")) {

            turnRelative(turnAngle, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(-turnAngle, turnSpeed);
            move(-14, moveSpeed);

        } else if (goldPos.equals("Right")) {

            turnRelative(-turnAngle, turnSpeed);
            move(-37, moveSpeed);
            /*turnRelative(turnAngle, turnSpeed);
            move(-8, moveSpeed);*/
        } else if (goldPos.equals("Center")) {

            move(-42, moveSpeed);
        }
    }

    public void hitGoldDepot() {

        String goldPos = "bad";

        //turn right to look at 2 minerals
        turnRelative(-lookAngle, turnSpeed);

        //figure out gold position
        timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: ", timer.seconds());

            goldPos = autoFindGold();
            telemetry.addData("Gold: ", goldPos);
            telemetry.update();

        }

        closeTfod();

        //turn back to starting position
        turnRelative(lookAngle, turnSpeed);

        //default to left if can't detect anything rip
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {

            turnRelative(27, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(-54, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(27, turnSpeed);
        } else if (goldPos.equals("Right")) {

            turnRelative(-27, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(54, turnSpeed);
            move(-37, moveSpeed);
            turnRelative(-27, turnSpeed);
        } else if (goldPos.equals("Center")) {

            move(-40, moveSpeed);
        }
    }

    public void hitGold() {

        String goldPos = "bad";

        //turn right to look at 2 minerals
        turnRelative(-lookAngle, turnSpeed);

        //figure out gold position
        timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: ", timer.seconds());

            goldPos = autoFindGold();

            telemetry.addData("Gold: ", goldPos);
            telemetry.update();

        }

        closeTfod();

        //turn back to starting position
        turnRelative(lookAngle, turnSpeed);

        //default to left if can't detect anything rip
        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {


        } else if (goldPos.equals("Right")) {


        } else if (goldPos.equals("Center")) {


        }
    }

    public void hitGold(int evaluator) {

        String goldPos = "bad";

        //turn right to look at 2 minerals
        turnRelative(-lookAngle, turnSpeed);

        //figure out gold position
        timer = new ElapsedTime();
        int direction = 1;
        while (opModeIsActive() && timer.seconds() < 5 && goldPos.equals("bad")) {
            telemetry.addData("Timer: ", timer.seconds());

            goldPos = autoFindGold(evaluator);

            if (evaluator == 3) {

                turnRelative(lookAngle * 2 * direction, turnSpeed);
                waitForMotors();
                direction *= -1;
            }

            telemetry.addData("Gold: ", goldPos);
            telemetry.update();

        }

        closeTfod();

        if (evaluator == 3) {

            turnRelative(lookAngle * direction * -1, turnSpeed);
        } else {

            turnRelative(lookAngle, turnSpeed);
        }

        if (goldPos.equals("bad")) {
            goldPos = "Left";
        }

        if (goldPos.equals("Left")) {


        } else if (goldPos.equals("Right")) {


        } else if (goldPos.equals("Center")) {


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

    public void blinker() {

        LEDStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void waitTime(int time) {
        try {

            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void shift() {

        moveSlidesTo(0, 0.1f);

        if (shifterServo.getPosition() == speed) {
            shifterServo.setPosition(stronk);
        } else if (shifterServo.getPosition() == stronk) {
            shifterServo.setPosition(speed);
        }
    }

   /* public void ejectTeamMarker() {

        moveSlidesAbsolute(1400, 0.2);
        moveHingeTo(20);
        for (int i = 0; i < 3; i++) {
            turnRelative(5, 2 * turnSpeed);
            turnRelative(-5, 2 * turnSpeed);
        }
    }*/

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
                    case 3:

                        goldPosition = evaluateRecognitionsV3(updatedRecognitions);
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

    private String evaluateRecognitionsV3(List<Recognition> recognitions) {

        String goldPosition = "bad";

        if (recognitions != null) {

            if (recognitions.size() == 2) {

                int goldMineralX = -1;
                int silverMineralX = -1;

                int hasGold = 0;

                for (Recognition r : recognitions) {

                    if (!r.getLabel().equals(LABEL_GOLD_MINERAL)) {

                        hasGold++;
                    }
                }

                if (hasGold == 1) {

                    for (Recognition r : recognitions) {

                        // bottom is left, right is bottom
                        if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {

                            goldMineralX = (int) r.getBottom();
                        } else if (r.getLabel().equals(LABEL_SILVER_MINERAL)) {

                            silverMineralX = (int) r.getBottom();
                        }
                    }

                    if (goldMineralX < silverMineralX) {

                        goldPosition = "Left";
                    } else if (silverMineralX > goldMineralX) {

                        goldPosition = "Right";
                    }
                } else if (hasGold == 2) {

                    double confidence1 = recognitions.get(0).getConfidence();
                    double confidence2 = recognitions.get(1).getConfidence();

                    if (confidence1 > confidence2) {

                        goldPosition = (recognitions.get(0).getBottom() < recognitions.get(1).getBottom()) ? "Left" : "Right";
                    } else if (confidence1 < confidence2) {

                        goldPosition = (recognitions.get(0).getBottom() > recognitions.get(1).getBottom()) ? "Left" : "Right";
                    }
                }
            }
        }

        return goldPosition;
    }

    public void updateNavTargets() {

        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    /*private String[] findMineralPositions(List<Recognition> list) {
        String[] minerals = new String[3];
        int goldX = -1;
        int silver1X = -1;
        int silver2X = -2;
        for (Recognition mineral : list) {
            if (mineral.getLabel().equals("LABEL_GOLD_MINERAL")) {
                goldX = (int) mineral.getBottom();
            }
            else if (silver1X == -1) {
                silver1X = (int) mineral.getBottom();
            }
            else {
                silver2X = (int) mineral.getBottom();
            }
        }

        //only see two silvers, gold is on left
        if (goldX == -1) {
            minerals[0] = "GOLD";
            minerals[1] = "SILVER";
            minerals[2] = "SILVER";
        }

        //see one of each
        if (silver2X == -1) {
            if (goldX < silver1X) {
                minerals[0] = "SILVER";
                minerals[1] = "GOLD";
                minerals[2] = "SILVER";
            } else {
                minerals[0] = "SILVER";
                minerals[1] = "SILVER";
                minerals[2] = "GOLD";
            }
        }
        return minerals;
    }

    private String getGoldPos(String[] mineralPositions) {
        String goldPos = "";
        if (mineralPositions[0].equals("GOLD")) {
            goldPos = "LEFT";
        }
        else if (mineralPositions[1].equals("GOLD")) {
            goldPos = "CENTER";
        }
        else if (mineralPositions[2].equals("GOLD")) {
            goldPos = "RIGHT";
        }
        return goldPos;
    }*/

    public void sample() {
        String[] minerals = new String[3];
        String goldPos = "bad";

        int goldX = -1;
        int silver1X = -1;
        int silver2X = -1;

        turnRelative(lookAngle, turnSpeed);
        timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 3 && goldPos.equals("bad")) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size() == 2) {
                    for (Recognition mineral : updatedRecognitions) {
                        if (mineral.getLabel().equals("LABEL_GOLD_MINERAL")) {
                            goldX = (int) mineral.getBottom();
                        } else if (silver1X == -1) {
                            silver1X = (int) mineral.getBottom();
                        } else {
                            silver2X = (int) mineral.getBottom();
                        }

                        //only see two silvers, gold is on left
                        if (goldX == -1) {
                            minerals[0] = "GOLD";
                            minerals[1] = "SILVER";
                            minerals[2] = "SILVER";
                        }

                        //see one of each
                        if (silver2X == -1) {
                            if (goldX < silver1X) {
                                minerals[0] = "SILVER";
                                minerals[1] = "GOLD";
                                minerals[2] = "SILVER";
                            } else {
                                minerals[0] = "SILVER";
                                minerals[1] = "SILVER";
                                minerals[2] = "GOLD";
                            }
                        }

                    }
                    String posString = minerals[0] + " " + minerals[1] + " " + minerals[2];
                    telemetry.addData("Positions", posString);
                    telemetry.addData("Gold Position", goldPos);
                    telemetry.update();
                }
            }
        }
    }
}