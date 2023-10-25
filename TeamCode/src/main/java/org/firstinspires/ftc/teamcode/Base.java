package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public abstract class Base extends LinearOpMode {
    public List<LynxModule> allHubs;
    public DcMotor fLeftMotor;
    public DcMotor fRightMotor;
    public DcMotor bLeftMotor;
    public DcMotor bRightMotor;
    public DcMotor sweeper;
    public DcMotor slideRight;
    public DcMotor slideLeft;

    public IMU imu;
    public void initHardware(){

        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        fLeftMotor = hardwareMap.get(DcMotor.class, "fLeft");
        bLeftMotor = hardwareMap.get(DcMotor.class, "fLeft");
        bRightMotor = hardwareMap.get(DcMotor.class, "fLeft");
        fRightMotor = hardwareMap.get(DcMotor.class, "fLeft");

        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        slideRight = hardwareMap.get(DcMotor.class, "right_slide");
        slideLeft = hardwareMap.get(DcMotor.class, "left_slide");



        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters( new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
                ));


    }

    public void driveFieldCentric(double drive, double strafe, double turn){


        double heading = Math.toRadians(getAngle());


        double rotVectorX = strafe*Math.cos(-heading) - drive*Math.sin(-heading);
        double rotVectorY = drive*Math.cos(-heading) + strafe*Math.sin(-heading);

        rotVectorX *= 1.1;

        double denominator = Math.max(Math.abs(rotVectorY) + Math.abs(rotVectorX) + Math.abs(turn), 1);
        double frontLeftPower = (rotVectorY + rotVectorX + turn) / denominator;
        double backLeftPower = (rotVectorY - rotVectorX + turn) / denominator;
        double frontRightPower = (rotVectorY - rotVectorY - turn) / denominator;
        double backRightPower = (rotVectorY + rotVectorX - turn) / denominator;

        fLeftMotor.setPower(frontLeftPower);
        bLeftMotor.setPower(backLeftPower);
        fRightMotor.setPower(frontRightPower);
        bRightMotor.setPower(backRightPower);



    }


    public void resetCache(){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }

    public double getAngle(){
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}

