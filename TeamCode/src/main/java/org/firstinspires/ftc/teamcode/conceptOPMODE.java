//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.util.Arrays;
//
//@TeleOp(name = "concept", group = "TeleOP")
//
//public class conceptOPMODE extends ExponentialFunctions {
//
//    public static enum buttons {a, b, x, y, start, back, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button};
//    private buttons[] pressed = new buttons[buttons.values().length];
//
//    public void runOpMode() throws InterruptedException {
//
//        super.runOpMode();
//        waitForStart();
//
//        while(!isStopRequested()) {
//
//            // collect the pressed buttons
//
//            byte[] pressed = gamepad1.toByteArray()
//
//            // trim off the null values on the end
//            pressed = Arrays.copyOfRange(pressed, 0, Arrays.asList(pressed).indexOf(null));
//
//            // actions based on buttons pressed
//            for (int button = 0; button < pressed.length; button++) {
//
//                switch (pressed[button]) {
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
//        }
//    }
//
////    public byte[] getGamepadByte() {
////
////        byte[] gamepad;
////
////        try {
////
////            gamepad = gamepad1.toByteArray();
////        } catch (Exception e) {
////
////
////        }
////
////        return
////    }
//    public static void main(String[] args) {
//
//
//    }
//}
