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

        while(isStopRequested()) {

            // collect the pressed buttons
            byte[] pressedByte = new byte[buttons.values().length];

            try {

                pressedByte = gamepad1.toByteArray();
            } catch(Exception e) {

                e.printStackTrace();
            }

            pressed = new int[pressedByte.length];

            int minPointer = 0;
            // map button values to pressed
            for (int buttonByte = 0; buttonByte < pressed.length; buttonByte++) {

                if (pressedByte[buttonByte] > 0)  {

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

            telemetry.addData("Buttons Pressed: ", Arrays.toString(pressed));
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
