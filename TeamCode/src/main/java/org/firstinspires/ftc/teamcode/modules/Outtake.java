package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Outtake extends SubsystemBase {
    public DcMotorEx arm;
    public CommandScheduler commandScheduler;

    public Outtake(DcMotorEx arm){
        this.arm = arm;
    }


    public void liftArm(int target){

    }




}
