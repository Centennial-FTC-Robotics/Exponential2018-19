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
            movementVectors[m].scale(3);
        }

        for (Vector v: movementVectors) {

            // telemetry variables
            double currentAngle = getRotationinDimension('Z');
            Vector orientationVector = new Vector(new double[] {Math.cos(currentAngle), Math.sin(currentAngle)});
            double targetAngle = standardPosAngle(v);
            double moveAngle = orientationVector.angleBetween(v) * getAngleDir(targetAngle, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("targetAngle: ", targetAngle);
            telemetry.addData("movement Angle: ", moveAngle);
            telemetry.update();

            move(v, 0.2f);
            waitForMotors();
            while(!gamepad1.a) {}
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

        // hopefully this will work now that I've corrected for the loss of direction when calculating angles between vectors
        //vectorMoveTest();
        double[][] testCoeff= {{8.5d,0d,0d,5.5d}};
       // {6d,.2d,1d,0d},{6d,.2d,2d,0d}, {6d,.2d,3d,0d}, {6d,.2d,4d,0d}, {6d,.2d,5d,0d}
        //move(24f, .4f);
        PIDTester(testCoeff);
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

    public static boolean VectorAbsAngleTester() {

        boolean testsPassed = true;

        for (int theta = 0; theta < 360; theta++) {

            Vector testV = new Vector(new double[] {Math.cos(Math.toRadians(theta)), Math.sin(Math.toRadians(theta))});

            // commented out bc only set standardPosAngle() to be static to test it, not gonna have a syntax error laying around now
//            if (Math.round(ExponentialFunctions.standardPosAngle(testV)) != theta) {
//
//                System.out.println("Input Angle: " + theta + " Output: " + Math.round(ExponentialFunctions.standardPosAngle(testV)));
//                testsPassed = false;
//            }
        }

        return testsPassed;
    }

    public static boolean vectorAngleTester() {

        boolean testsPassed = true;

        Vector i = new Vector(new double[] {1, 0});

        for (int d = -180; d < 180; d++) {

            Vector v = new Vector(new double[] {Math.cos(Math.toRadians(d)), Math.sin(Math.toRadians(d))});

            if ((i.angleBetween(v) * getAngleDir(d, 0)) != d) {

                testsPassed = false;
            }
        }

        return testsPassed;
    }

    public void PIDTester(double[][] coefficients) {

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
    }

    public static void main(String[] args) {

        Tester t = new Tester();

        System.out.println(VectorAbsAngleTester()); // Works!

        System.out.println(VectorAbsAngleTester()); // not tested

        double[][] PIDCoefficients = new double[][] {
                {0, 0, 0, 0},
                {1, 1, 1, 1},
                {0.5, 0.5, 0.5, 0.5}
        };

        //t.PIDTester(, PIDCoefficients);
    }
}
