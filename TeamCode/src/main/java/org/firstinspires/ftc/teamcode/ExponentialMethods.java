package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class ExponentialMethods extends ExponentialHardware {

    //motors
    private final DcMotor[] leftDriveMotors = {lmotor0, lmotor1};
    private final DcMotor[] rightDriveMotors = {rmotor0, rmotor1};
    private final DcMotor[] driveMotors = {lmotor0, lmotor1, rmotor0, rmotor1};

    //for reference
    private final int driveTicksPerRev = 560;
    private final int driveSprocket = 24;
    private final int wheelSprocket = 22;
    private final int wheelDiameterIn = 4;

    //tensor flow and vuforia stuff
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    private VuforiaLocalizer vuforia; //Vuforia localization engine
    private TFObjectDetector tfod; //Tensor Flow Object Detection engine

    //movement based on speeds
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

    //currently in inches
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

    private void initVuforia() {
        //create parameter object and pass it to create Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

    public void runAuto() {
        autoInit();
        autoFindGold();
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

    //returns left, right, or center based on position of gold
    public String autoFindGold() {

        //added:
        String goldPosition = "";

        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            //gets x positions for each mineral detected
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
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
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        //added:
        return goldPosition;
    }
}
