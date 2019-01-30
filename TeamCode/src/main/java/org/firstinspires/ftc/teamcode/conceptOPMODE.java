package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "concept", group = "TeleOP")

public class conceptOPMODE extends ExponentialFunctions {

    public enum buttons {a, b, x, y, start, back, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button};
    private int[] pressed;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        waitForStart();

        while(opModeIsActive()) {

            // collect the pressed buttons
            String gamepadState = gamepad1.toString();



            /* user, left_x, left_y, right_x, right_y, left_trigger, right_trigger, dpad_up, dpad_down, dpad_left, dpad_up,
               a, b, x, y, start, left_bumper, right_bumper, left stick button, right stick button
            */

            telemetry.addData("Gamepad ", gamepad1.toString());
            telemetry.update();

            // actions based on buttons pressed
//            for (int button = 0; button < pressed.length; button++) {
//
//                buttons currentPressed = buttons.values()[pressed[button]];
//
//                switch (currentPressed) {
//                    case a:
//                        break;
//                    case b:
//                        break;
//                    case x:
//                        break;
//                    case y:
//                        break;
//                    case start:
//                        break;
//                    case back:
//                        break;
//                    case dpad_up:
//                        break;
//                    case dpad_down:
//                        break;
//                    case dpad_left:
//                        break;
//                    case dpad_right:
//                        break;
//                    case left_bumper:
//                        break;
//                    case right_bumper:
//                        break;
//                    case left_stick_button:
//                        break;
//                    case right_stick_button:
//                        break;
//                }
//            }
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
