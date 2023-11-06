package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Objects;

public abstract class Base extends LinearOpMode {
    public List<LynxModule> allHubs;
    public DcMotor fLeftMotor;
    public DcMotor fRightMotor;
    public DcMotor bLeftMotor;
    public DcMotor bRightMotor;

    public DcMotor sweeper;

    public DcMotor slideRight;

    public DcMotor slideLeft;

    public DcMotor hanger;

    public Servo launcher;

    public double p=0, i=0, d=0, f=0;

    public PIDController slidePid = new PIDController(0, 0, 0);

    public String state = "down";

    public IMU imu;

    public int HIGH_DEPO_POS = 1000, MID_DEPO_POS = 700, LOW_DEPO_POS = 500;
    public void initHardware(){

        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Motors
        fLeftMotor = hardwareMap.get(DcMotor.class, "fLeft");
        bLeftMotor = hardwareMap.get(DcMotor.class, "fLeft");
        bRightMotor = hardwareMap.get(DcMotor.class, "fLeft");
        fRightMotor = hardwareMap.get(DcMotor.class, "fLeft");

        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        hanger = hardwareMap.get(DcMotor.class, "hanger");

        //Servos
        launcher = hardwareMap.get(Servo.class, "launch");




        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters( new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
                ));




    }

    public void depositUp(double targetPos){
        double slidePower = slidePid.calculate(targetPos - slideLeft.getCurrentPosition()) + f;
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    public void updateArmPos(){
        if(state.equals("down")){
            if(slideLeft.getCurrentPosition() > 10){
                depositUp(0);
            }else{
                slideLeft.setPower(0);
                slideRight.setPower(0);
            }
        }else if(state.equals("low")){
            depositUp(LOW_DEPO_POS);
        }else if(state.equals("mid")){
            depositUp(MID_DEPO_POS);
        }else if(state.equals("high")){
            depositUp(HIGH_DEPO_POS);
        }
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

    public void initSlidePID(double k_p, double k_i, double k_d){
        slidePid.setPID(k_p, k_i, k_d);
    }


    public void resetCache(){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    public double getAngle(){
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }
}

