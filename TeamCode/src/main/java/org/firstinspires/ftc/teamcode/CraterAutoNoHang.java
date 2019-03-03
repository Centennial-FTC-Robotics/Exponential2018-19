package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="CraterAutoNoHang", group="TeleOp")

public class CraterAutoNoHang extends ExponentialFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        initVision();
        initializeIMU();
        waitForStart();
        //hitGoldCrater();
    }
}
