package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.*;

public class TaskManager {

    private ArrayList<Task> tasks;
    private ArrayList<ElapsedTime> times;

    TaskManager() {
        tasks = new ArrayList<Task>();

    }

    TaskManager(TaskManager taskSet) {
        this();
        tasks.addAll(taskSet.getTasks());
        times = new ArrayList<ElapsedTime>();

        for (Task task: tasks) {

            times.add(new ElapsedTime());
        }
    }

    TaskManager(ArrayList<Task> newTasks) {
        this();
        tasks.addAll(newTasks);
        times = new ArrayList<ElapsedTime>();

        for (Task task: tasks) {

            times.add(new ElapsedTime());
        }
    }

    public void startTasks() {

        for (int task = 0; task < tasks.size(); task++) {

            startTask(task);
        }
    }

    public void startTask(Task task) {

        startTask(tasks.indexOf(task));
    }

    public void startTask(int taskIndex) {

        if (taskIndex < tasks.size()) {

            Task task = tasks.get(taskIndex);

            if (!task.isActive()) {

                if (task.motorType().equals("Servo")) {

                    Servo servo = (Servo) task.getMotor();

                    servo.setPosition(task.getMotorAction());
                } else if (task.motorType().equals("DcMotor")) {

                    DcMotor motor = (DcMotor) task.getMotor();

                    if (task.isPos()) {

                        motor.setTargetPosition(Range.clip((int) task.getMotorAction(), -1, 1));
                    } else {

                        motor.setPower(task.getMotorAction());
                    }
                }

                task.activate();
            }
        }
    }

    public void checkTasks() {

        for (int task = 0; task < tasks.size(); task++) {

            if (!checkTask(task)) {

                task--;
            }
        }
    }

    public void checkTask(Task task) {

        checkTask(tasks.indexOf(task));
    }

    public boolean checkTask(int taskIndex) {

        if (taskIndex < tasks.size()) {

            Task task = tasks.get(taskIndex);
            ElapsedTime taskTimer = times.get(taskIndex);
            double timeRunning = taskTimer.time() - taskTimer.startTime();

            if (timeRunning < task.getTime()) {
                if (task.isActive()) {

                    if (task.motorType().equals("Servo")) {

                        Servo servo = (Servo) task.getMotor();

                        servo.setPosition(task.getMotorAction());
                    } else if (task.motorType().equals("DcMotor")) {

                        DcMotor motor = (DcMotor) task.getMotor();

                        if (task.isPos()) {

                            motor.setTargetPosition((int) task.getMotorAction());
                        } else {

                            motor.setPower(task.getMotorAction());
                        }
                    }
                }

                return true;
            } else {

                tasks.remove(taskIndex);
                times.remove(taskIndex);

                return false;
            }
        }

        return true;
    }

    public ArrayList<Task> getTasks() {

        return tasks;
    }
}