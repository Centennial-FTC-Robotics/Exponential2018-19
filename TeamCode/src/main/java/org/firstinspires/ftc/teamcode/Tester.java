package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name="Tester", group="TeleOP")

public class Tester extends ExponentialFunctions {

    public void relativeTurnDriver() {

        int[] angles = {90, -100, 50, 4, 16, -76, 36, 180};

        for (int angle = 0; angle < angles.length && opModeIsActive(); angle++) {

            telemetry.addData("current relative angle: ", angles[angle]);
            telemetry.update();
            turnRelative(angles[angle]);
            while (!gamepad1.a && opModeIsActive()) {};
        }
    }

    public void linearMoveTest() {

        int[] distances = {12, 2, -5, 6, 8, -14, 24, -9, 0};

        for (int distance = 0; distance < distances.length && opModeIsActive(); distance++) {

            telemetry.addData("current distance: ", distances[distance]);
            telemetry.addData("targetPosition", convertInchToEncoder(distances[distance]));
            telemetry.addData("lmotor0: ", lmotor0.getCurrentPosition());
            telemetry.addData("lmotor1: ", lmotor1.getCurrentPosition());
            telemetry.addData("rmotor0: ", rmotor0.getCurrentPosition());
            telemetry.addData("rmotor1: ", rmotor1.getCurrentPosition());
            telemetry.update();
            move((float) distances[distance], (float) 0.2);
            while(!gamepad1.a && opModeIsActive());
        }
    }

    public void moveModifiedTest() {
        int[] distances = {36, -36, 36, -36, 36, -36, 36, -36, 36, -36};
        //int[] distances = {48, -48, 24, -24, 12, -12, 6, -6, 3, -3};

        for (int distance = 0; distance < distances.length && opModeIsActive(); distance++) {

            telemetry.addData("current distance: ", distances[distance]);
            telemetry.addData("targetPosition", convertInchToEncoder(distances[distance]));
            telemetry.addData("lmotor0: ", lmotor0.getCurrentPosition());
            telemetry.addData("lmotor1: ", lmotor1.getCurrentPosition());
            telemetry.addData("rmotor0: ", rmotor0.getCurrentPosition());
            telemetry.addData("rmotor1: ", rmotor1.getCurrentPosition());
            telemetry.update();
            moveModified((float) distances[distance], (float) 0.5);
            while(!gamepad1.a && opModeIsActive());
        }
    }

    public void sampleTest() {
        initVision();
        String goldPos = findGold();
        if (goldPos.equals("LEFT")) {
            turnRelative(45);
        }
        else if (goldPos.equals("RIGHT")) {
            turnRelative(-45);
        }
        moveModified(-20, 0.4f);
    }

    public void servoPosTesting(Servo testServo) {


        while (opModeIsActive()) {

            testServo.setPosition((gamepad1.right_stick_y + 1) / 2);
            telemetry.addData("Current Servo Pos: ", testServo.getPosition());
            telemetry.update();

        }
    }

    public String[] findGoldEvaluationTester() {

        int evaluatorCount =  3;
        String[] results = new String[evaluatorCount];

        for (int e = 0; e < evaluatorCount; e++) {

            results[e] = autoFindGold(e);
        }

        return results;
    }

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        waitForStart();
        //relativeTurnDriver();
        //moveModifiedTest();
        sampleTest();
        //move(12);
        //turnRelative(90, .2)
//        while (opModeIsActive()) {
//            telemetry.addData("currentAngle", getRotationinDimension('Z'));
//            telemetry.update();
//        }

        //relativeTurnDriver();
        //linearMoveTest();

        //double[][] testCoeff= {{8.5d,0d,0d,5.5d}};
       // {6d,.2d,1d,0d},{6d,.2d,2d,0d}, {6d,.2d,3d,0d}, {6d,.2d,4d,0d}, {6d,.2d,5d,0d}
        //move(24f, .4f);
        //PIDTester(testCoeff);
        //servoPosTesting(shifterServo);
//        double[][] motorVelocities = motorVelTesting(10);
//
//        for (int v = 0; v < motorVelocities.length; v++) {
//            telemetry.addData("Velocities", Arrays.toString(motorVelocities[v]));
//        }
//
//        telemetry.update();
//        while (!gamepad1.a) {};
        //blinker();
    }


    /*public void PIDTester(double[][] coefficients) {

        boolean testsPassed = true;
        //move(24f,.62f);
        //while (!gamepad1.a && opModeIsActive()) {};

        for (int PID = 0; PID < coefficients.length && coefficients[PID].length == 4; PID++) {
            while (!gamepad1.a && opModeIsActive()) {};
            setPID(lmotor0, coefficients[PID][0], coefficients[PID][1], coefficients[PID][2], coefficients[PID][3]);
            setPID(lmotor1, coefficients[PID][0], coefficients[PID][1], coefficients[PID][2], coefficients[PID][3]);
            setPID(rmotor0, coefficients[PID][0], coefficients[PID][1], coefficients[PID][2], coefficients[PID][3]);
            setPID(rmotor1, coefficients[PID][0], coefficients[PID][1], coefficients[PID][2], coefficients[PID][3]);
            lmotor0.setTargetPositionTolerance(20);
            lmotor1.setTargetPositionTolerance(20);
            rmotor0.setTargetPositionTolerance(20);
            rmotor1.setTargetPositionTolerance(20);



            telemetry.addData("test num: ", PID);
            telemetry.addData("target: ", lmotor0.getTargetPosition());
            telemetry.addData("lmotor0: ", lmotor0.getTargetPositionTolerance());

            telemetry.update();
            move(30f,.6f);

        }
    }*/

    public static void main(String[] args) {

    }
}
