package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class T1AutonTest extends Base{

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while(timer.milliseconds() <= 3000){
            driveFieldCentric(0.5, 0.3, 0);
        }


        timer.reset();
        while(timer.milliseconds() <= 4000){
            driveFieldCentric(0.2, 0, 0);
        }

        timer.reset();
        while(timer.milliseconds() <= 3000){
            driveFieldCentric(0.5, 0, 0);
        }

        timer.reset();
        while(timer.milliseconds() <= 3500){
            driveFieldCentric(0.3, 0, 0);
        }
    }
}
