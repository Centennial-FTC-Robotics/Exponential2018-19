package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class Task {

    private HardwareDevice motor;
    private double action;
    private double time;

    private boolean active = false;
    private boolean isPos;

    Task(DcMotor newMotor, double newAction, double newTime, boolean newIsPos) {

        motor = newMotor;
        action = newAction;
        time = newTime;

        isPos = newIsPos;
    }

    Task(Servo newServo, double newAction, double newTime) {

        motor = newServo;
        action = newAction;
        time = newTime;

        isPos = true;
    }

    public HardwareDevice getMotor() {

        return motor;
    }

    public double getMotorAction() {

        return action;
    }

    public double getTime() {

        return time;
    }

    public boolean isPos() {

        return isPos;
    }

    public boolean isActive() {

        return active;
    }

    public String motorType() {

        boolean isDcMotor = true;
        try {

            motor = (DcMotor) motor;
        } catch (ClassCastException e) {

            isDcMotor = false;
        }

        String motorType = (isDcMotor) ? "DcMotor" : "Servo";
        //motorType = ((DcMotor) motor).getMotorType().toString();
        return motorType;
    }

    protected void activate() {

        active = true;
    }
}