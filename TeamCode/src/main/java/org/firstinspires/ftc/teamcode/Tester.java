package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

    public void vectorMoveTest() {

        double[][] movements = new double[][] {
                {2, 2},
                {-2, 2},
                {-2, -2},
                {2, -2}
        };

        Vector[] movementVectors = new Vector[movements.length];

        for (int m = 0; m < movements.length; m++) {

            movementVectors[m] = new Vector(movements[m]);
        }

        for (Vector v: movementVectors) {

            move(v, 0.2f);
            waitForMotors();
        }
    }

    public void servoPosTesting(Servo testServo) {


        while (opModeIsActive()) {

            testServo.setPosition((gamepad1.right_stick_y + 1) / 2);
            telemetry.addData("Current Servo Pos: ", testServo.getPosition());
            telemetry.update();

        }
    }

    public double[][] motorVelTesting(int intervalCount) {

        double[][] velocities = new double[intervalCount][2];

        // initial reset
        moveHingeTo(0);
        while (lHingeMotor.isBusy() && rHingeMotor.isBusy() && opModeIsActive()) {};

        for (int i = 1; i <= intervalCount; i++) {

            // initial position
            int x0 = lHingeMotor.getCurrentPosition();

            // actual movement and measurement
            moveHinge((float) (i / ((double) intervalCount)));
            moveIntakeArm(1);

            waitTime(1000);

            int x1 = lHingeMotor.getCurrentPosition();

            // store velocity

            velocities[i - 1] = new double[] {(i / ((double) intervalCount)), (x1 - x0)};

            // reset
            moveHinge(0);
            moveHingeTo(0);
            while (lHingeMotor.isBusy() && rHingeMotor.isBusy() && opModeIsActive()) {};
        }

        return velocities;
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

        //move(12);
        //turnRelative(90, .2)
//        while (opModeIsActive()) {
//            telemetry.addData("currentAngle", getRotationinDimension('Z'));
//            telemetry.update();
//        }

        //relativeTurnDriver();
        //linearMoveTest();
        vectorMoveTest();

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
}
