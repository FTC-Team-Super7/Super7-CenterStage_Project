package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

@Autonomous
public class T2_Auto extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareAuto();
        resetCache();
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        timer.reset();
        while(timer.milliseconds() < 400){
            resetCache();
            driveFieldCentricAuto(0.4, 0.3, 0);
        }
        stopDrive();

//        timer.reset();
//        while(timer.milliseconds() < 400){
//            resetCache();
//            driveFieldCentricAuto(0, 0.3, 0);
//        }
//        stopDrive();

    }
}
