package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TaskManager {

    private ArrayList<Task> tasks;
    private ArrayList<ElapsedTime> times;

    TaskManager() {
        tasks = new ArrayList<Task>();

    }

    TaskManager(TaskManager taskSet) {
        this();
        tasks.addAll(taskSet.getTasks());

    }

    TaskManager(ArrayList<Task> newTasks) {
        this();
        tasks.addAll(newTasks);

    }

    private void checkTasks() {

        for (int task = 0; task < tasks.size(); task++) {

            ElapsedTime taskTimer = times.get(task);
            double timeElapsed = taskTimer.time() - taskTimer.startTime();

            if (tasks.get(task).getTime() <= timeElapsed) {

                tasks.remove(task);
                times.remove(tasks);
                task--;
            } else {

                Task motorTask = tasks.get(task);

                if (motorTask.motorType().equals("Servo")) {

                    Servo servo = (Servo) motorTask.getMotor();
                } else if (motorTask.motorType().equals("DcMotor")) {

                    DcMotor motor = (DcMotor) motorTask.getMotor();
                }
            }
        }
    }

    public ArrayList<Task> getTasks() {

        return tasks;
    }
}