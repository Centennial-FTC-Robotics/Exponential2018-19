package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name="Tester", group="CraterAuto")

public class Tester extends ExponentialFunctions {

    public void relativeTurnDriver() {

        int[] angles = {90, -100, 50, 4, 16, -76, 36, 180};

        for (int angle = 0; angle < angles.length && opModeIsActive(); angle++) {

            telemetry.addData("current relative angle: ", angles[angle]);
            telemetry.update();
            turnRelative(angles[angle], 0.2);
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

    public void servoPosTesting(Servo testServo) {


        while (opModeIsActive()) {

            testServo.setPosition((gamepad1.right_stick_y + 1) / 2);
            telemetry.addData("Current Servo Pos: ", testServo.getPosition());
            telemetry.update();

        }
    }

    public double[][] motorVelTesting(int intervalCount) {

        double[][] velocities = new double[intervalCount][2];

        for (double i = (1.0 / intervalCount); i <= 1; i += (1.0 / intervalCount)) {

            // reset
            moveHingeTo(0);
            while (lHingeMotor.isBusy() && rHingeMotor.isBusy() && opModeIsActive()) {};

            // actual movement and measurement
            lHingeMotor.setPower(i);
            rHingeMotor.setPower(i);

            velocities[(int) (i * intervalCount)] = new double[] {(1 / intervalCount),(int) ((lHingeMotor.getVelocity() + rHingeMotor.getVelocity()) / 2.0)};

            lHingeMotor.setPower(0);
            rHingeMotor.setPower(0);
        }

        telemetry.addData("Velocities: ", Arrays.toString(velocities));

        return velocities;
    }

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        //move(12);
        //turnRelative(90, .2)
//        while (opModeIsActive()) {
//            telemetry.addData("currentAngle", getRotationinDimension('Z'));
//            telemetry.update();
//        }

        //relativeTurnDriver();
        //linearMoveTest();

        //servoPosTesting(shifterServo);
        //double[][] motorVelocities = motorVelTesting(10);
        //telemetry.addData("Velocities: ", Arrays.toString(motorVelocities));
        //blinker();
    }
}
