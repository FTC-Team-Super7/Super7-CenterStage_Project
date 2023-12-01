package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Objects;

public abstract class Base extends LinearOpMode {
    public List<LynxModule> allHubs;
    Motor fLeftMotor;
    Motor bLeftMotor;
    Motor bRightMotor;
    Motor fRightMotor;
    Motor hanger;
    Motor arm;
    Servo launcher;
    Servo pivot;
    Servo leftClaw;
    Servo rightClaw;

    DistanceSensor distance_sensor;

    public IMU imu;



    public double BOX_INIT_POS = 0.4, BOX_INTAKE_POS = 0.65, BOX_RETRACT_POS = 0.5, BOX_MID_POS = 0.45;
    public double LAUNCHER_INIT_POS = 0, LAUNCHER_SHOOT_POS = 0.5;

    public double LEFT_CLAW_OPEN = 0, LEFT_CLAW_CLOSE = 0.28, RIGHT_CLAW_OPEN = 0.65, RIGHT_CLAW_CLOSE = 0.06;














    public int HIGH_DEPO_POS = 1000, MID_DEPO_POS = 700, LOW_DEPO_POS = 500;
    public Drive dt;
    public void initHardware(){
        //Manual Bulk Caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        //Motors
         fLeftMotor = new Motor(hardwareMap, "fLeft", false);
         bLeftMotor = new Motor(hardwareMap, "bLeft", false);
         bRightMotor = new Motor(hardwareMap, "bRight", false);
         fRightMotor = new Motor(hardwareMap, "fRight", false);
         hanger = new Motor(hardwareMap, "hanger", false);
         arm = new Motor(hardwareMap, "arm", true);

         bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         //Servos
         launcher = hardwareMap.get(Servo.class, "launcher");
         pivot = hardwareMap.get(Servo.class, "pivot");
         leftClaw = hardwareMap.get(Servo.class, "leftClaw");
         rightClaw = hardwareMap.get(Servo.class, "rightClaw");

         //Sensors
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance");


        //Initalization Movements
        launcher.setPosition(LAUNCHER_INIT_POS);
        pivot.setPosition(0.6);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        rightClaw.setPosition(RIGHT_CLAW_OPEN);





        //Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters( new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
                ));
        resetYaw();


    }

    public void initHardwareAuto(){
        //Manual Bulk Caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        //Motors
        fLeftMotor = new Motor(hardwareMap, "fLeft", true);
        bLeftMotor = new Motor(hardwareMap, "bLeft", true);
        bRightMotor = new Motor(hardwareMap, "bRight", true);
        fRightMotor = new Motor(hardwareMap, "fRight", true);
        hanger = new Motor(hardwareMap, "hanger", false);
        arm = new Motor(hardwareMap, "arm", true);

        bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        launcher = hardwareMap.get(Servo.class, "launcher");
        pivot = hardwareMap.get(Servo.class, "pivot");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        //Sensors
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance");


        //Initalization Movements
        launcher.setPosition(LAUNCHER_INIT_POS);
        pivot.setPosition(0.6);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        rightClaw.setPosition(RIGHT_CLAW_OPEN);





        //Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters( new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
                ));
        resetYaw();


    }


    //Drive Functions
    public void driveFieldCentric(double drive, double strafe, double turn, double speedCap){


        double heading = Math.toRadians(getAngle());


        double rotVectorX = strafe*Math.cos(-heading) - drive*Math.sin(-heading);
        double rotVectorY = drive*Math.cos(-heading) + strafe*Math.sin(-heading);

        rotVectorX *= 1.1;

        double denominator = Math.max(Math.abs(rotVectorY) + Math.abs(rotVectorX) + Math.abs(turn), 1);
        double frontLeftPower = (rotVectorY + rotVectorX + turn) / denominator;
        double backLeftPower = (rotVectorY - rotVectorX + turn) / denominator;
        double frontRightPower = (rotVectorY - rotVectorX - turn) / denominator;
        double backRightPower = (rotVectorY + rotVectorX - turn) / denominator;

        fLeftMotor.setPower(frontLeftPower * speedCap);
        bLeftMotor.setPower(backLeftPower * speedCap);
        fRightMotor.setPower(frontRightPower * speedCap);
        bRightMotor.setPower(backRightPower * speedCap);



    }

    public void driveRobotCentric(double drive, double strafe, double turn, double speedCap){
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (drive + strafe + turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;

        fLeftMotor.setPower(frontLeftPower * speedCap);
        bLeftMotor.setPower(backLeftPower * speedCap);
        fRightMotor.setPower(frontRightPower * speedCap);
        bRightMotor.setPower(backRightPower * speedCap);
    }

    public void driveFieldCentricAuto(double drive, double strafe, double angle){
        double heading = Math.toRadians(getAngle());


        double rotVectorX = strafe*Math.cos(-heading) - drive*Math.sin(-heading);
        double rotVectorY = drive*Math.cos(-heading) + strafe*Math.sin(-heading);
        double angleError = normalizeAngle(angle - getAngle());
        double anglePow = 0.01 * angleError;

        rotVectorX *= 1.1;

        double denominator = Math.max(Math.abs(rotVectorY) + Math.abs(rotVectorX) + Math.abs(anglePow), 1);
        double frontLeftPower = (rotVectorY + rotVectorX + anglePow) / denominator;
        double backLeftPower = (rotVectorY - rotVectorX + anglePow) / denominator;
        double frontRightPower = (rotVectorY - rotVectorX - anglePow) / denominator;
        double backRightPower = (rotVectorY + rotVectorX - anglePow) / denominator;

        fLeftMotor.setPower(frontLeftPower);
        bLeftMotor.setPower(backLeftPower);
        fRightMotor.setPower(frontRightPower);
        bRightMotor.setPower(backRightPower);
    }

    public void stopDrive(){
        fLeftMotor.setPower(0);
        bLeftMotor.setPower(0);
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
    }

    public double normalizeAngle(double rawAngle) {
        double scaledAngle = rawAngle % 360;
        if (scaledAngle < 0) {
            scaledAngle += 360;
        }

        if (scaledAngle > 180) {
            scaledAngle -= 360;
        }

        return scaledAngle;
    }



    //Clears Bulk Read Cache
    public void resetCache(){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }

    //IMU Functions
    public void resetYaw(){
        imu.resetYaw();
    }

    public double getAngle(){
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;
    }

    //Teleop Functions
    public void autoGrab(){
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
    }

    public boolean sensePixel(){
        return distance_sensor.getDistance((DistanceUnit.CM)) < 10;
    }


}

