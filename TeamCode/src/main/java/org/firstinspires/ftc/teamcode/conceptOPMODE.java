package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "concept", group = "TeleOP")

public class conceptOPMODE extends ExponentialFunctions {

    public enum buttons {a, b, x, y, start, back, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button};
    private ArrayList<buttons> pressed;
    private double[] leftAnalog;
    private double[] rightAnalog;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        pressed = new ArrayList<buttons>();

        waitForStart();

        while(opModeIsActive()) {

            // collect the pressed buttons
            String gamepadState = gamepad1.toString();

            leftAnalog = new double[] {gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger};
            rightAnalog = new double[] {gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_trigger};

            for (buttons button: buttons.values()) {

                if (gamepadState.contains(button.toString().toLowerCase()) || gamepadState.contains(button.toString().toLowerCase().replace("_", " "))) {

                    pressed.add(button);
                }
            }

            /* user, left_x, left_y, right_x, right_y, left_trigger, right_trigger, dpad_up, dpad_down, dpad_left, dpad_up,
               a, b, x, y, start, left_bumper, right_bumper, left stick button, right stick button
            */

            telemetry.addData("Gamepad ", pressed.toString());
            telemetry.update();

            // actions based on buttons pressed
            for (buttons button: pressed) {

                switch (button) {
                    case a:
                        break;
                    case b:
                        break;
                    case x:
                        break;
                    case y:
                        break;
                    case start:
                        break;
                    case back:
                        break;
                    case dpad_up:
                        break;
                    case dpad_down:
                        break;
                    case dpad_left:
                        break;
                    case dpad_right:
                        break;
                    case left_bumper:
                        break;
                    case right_bumper:
                        break;
                    case left_stick_button:
                        break;
                    case right_stick_button:
                        break;
                }
            }

            pressed.clear();
        }
    }

    public byte[] getGamepadByte() {

        byte[] gamepad = new byte[0];

        try {

            gamepad = gamepad1.toByteArray();
        } catch (Exception e) {

            e.printStackTrace();
        }

        return gamepad;
    }

    public static void buttonSorter() {

        int[] pressed = {0, 0, 0, 0, 0, 1, 0, 0, 0, 1};

        int minPointer = 0;
        // map button values to pressed
        for (int buttonByte = 0; buttonByte < pressed.length; buttonByte++) {

            if (pressed[buttonByte] > 0) {

                pressed[minPointer] = buttonByte;
                minPointer++;
            } else {

                pressed[buttonByte] = -1;
            }
        }

        int index = 0;

        while (pressed[index] >= 0) {

            index++;
        }

        pressed = Arrays.copyOfRange(pressed, 0, index);

        System.out.println(Arrays.toString(pressed));
    }



    public static void main(String[] args) {


    }
}
