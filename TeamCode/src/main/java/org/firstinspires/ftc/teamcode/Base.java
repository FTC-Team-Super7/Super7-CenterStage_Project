package org.firstinspires.ftc.teamcode;



import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public abstract class Base extends LinearOpMode {
    public List<LynxModule> allHubs;
    Motor fLeftMotor;
    Motor bLeftMotor;
    Motor bRightMotor;
    Motor fRightMotor;
    Motor hanger;

    public DcMotorEx arm;
    public DcMotorEx arm2;

    CRServo launcher;
    Servo pivot;
    Servo leftClaw;
    Servo rightClaw;

    Servo droneAngle;

    DistanceSensor distance_sensor_right;
    DistanceSensor distance_sensor_left;
    DistanceSensor grabber_distance;

    public IMU imu;

    public PIDController controller;



    public double BOX_INIT_POS = 0.4, BOX_INTAKE_POS = 0.65, BOX_RETRACT_POS = 0.5, BOX_MID_POS = 0.45;
    public double LAUNCHER_INIT_POS = 0, LAUNCHER_SHOOT_POS = 0.5;

    public double LEFT_CLAW_OPEN = 0.48, LEFT_CLAW_CLOSE = 0.07, RIGHT_CLAW_OPEN = 0, RIGHT_CLAW_CLOSE = 0.29;














    public int HIGH_DEPO_POS = 1000, MID_DEPO_POS = 700, LOW_DEPO_POS = 500;
    public Drive dt;
    public void initHardware(OpMode opMode){
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
         arm = hardwareMap.get(DcMotorEx.class, "arm");
         arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
         droneAngle = hardwareMap.get(Servo.class, "angle");




         bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(0.02, 0, 0.001);

        //IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters( new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
                ));
        resetYaw();

        dt = new Drive(fLeftMotor, fRightMotor, bLeftMotor, bRightMotor, imu, opMode);

         //Servos
         launcher = hardwareMap.get(CRServo.class, "launcher");
         pivot = hardwareMap.get(Servo.class, "pivot");
         leftClaw = hardwareMap.get(Servo.class, "leftClaw");
         rightClaw = hardwareMap.get(Servo.class, "rightClaw");


         //Sensors
        distance_sensor_left = hardwareMap.get(DistanceSensor.class, "distance");
        distance_sensor_right = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        grabber_distance = hardwareMap.get(DistanceSensor.class, "grabber_distance");


        //Initalization Movements

        pivot.setPosition(0.82);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        droneAngle.setPosition(0.45);








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


        bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        launcher = hardwareMap.get(CRServo.class, "launcher");
        pivot = hardwareMap.get(Servo.class, "pivot");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        //Sensors
        distance_sensor_right = hardwareMap.get(DistanceSensor.class, "distance");


        //Initalization Movements

        pivot.setPosition(0.8);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);





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
    public void driveFieldCentric(double drive, double strafe, double turn, double speedCap, double initAng){


        double heading = Math.toRadians(getAngle() + initAng);


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

    public void driveFieldCentricNormal(double drive, double strafe, double turn, double speedCap){


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
    // Very similar to the carrot chasing algo (https://arxiv.org/abs/2012.13227)
    public void ChaseTheCarrot(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout) {
        ElapsedTime time = new ElapsedTime();
        ElapsedTime armTimer = new ElapsedTime();
        resetCache();
        dt.updatePosition();
        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        armTimer.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(dt.getX() - wp.get(wp.size() - 1).xP) > error
                || Math.abs(dt.getY() - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout) {
            dt.updatePosition();
            resetCache();

            double x = dt.getX();
            double y = dt.getY();
            double theta = getAngle();

            if (dt.getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                dt.updatePosition();
                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            //driveFieldCentric(-yPow, -turnPow, xPow);
            telemetry.addData("target: ", destPt);
            telemetry.update();
            driveFieldCentricNormal(xPow, -yPow, -turnPow, 1);

        }
        stopDrive();
    }

    public void ChaseTheCarrotConstantHeading(ArrayList<Point> wp,
                                              int switchTolerance,
                                              int skip,
                                              double heading,
                                              double error,
                                              double angleError,
                                              double normalMovementConstant,
                                              double finalMovementConstant,
                                              double turnConstant,
                                              double movementD,
                                              double turnD,
                                              double timeout){
        ChaseTheCarrot(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }
    public void ChaseTheCarrotSplineHeading(ArrayList<Point> wp,
                                            int switchTolerance,
                                            int skip,
                                            double error,
                                            double angleError,
                                            double normalMovementConstant,
                                            double finalMovementConstant,
                                            double turnConstant,
                                            double movementD,
                                            double turnD,
                                            double timeout){
        ChaseTheCarrot(wp, switchTolerance, skip,  true, false, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }

    public void ChaseTheCarrotSplineHeadingInverted(ArrayList<Point> wp,
                                                    int switchTolerance,
                                                    int skip,
                                                    double error,
                                                    double angleError,
                                                    double normalMovementConstant,
                                                    double finalMovementConstant,
                                                    double turnConstant,
                                                    double movementD,
                                                    double turnD,
                                                    double timeout){
        ChaseTheCarrot(wp, switchTolerance, skip, true, true, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }
    public void ChaseTheCarrotCustomHeading(ArrayList<Point> wp,
                                            int switchTolerance,
                                            int skip,
                                            double error,
                                            double angleError,
                                            double normalMovementConstant,
                                            double finalMovementConstant,
                                            double turnConstant,
                                            double movementD,
                                            double turnD,
                                            double timeout){
        ChaseTheCarrot(wp, switchTolerance, skip,  false, false, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }



    public void moveToPosition(double targetX, double targetY, double targetAngle, double MOE, double angleMOE, double maxTime, double speedCap){
        dt.updatePosition();
        ElapsedTime timer = new ElapsedTime();
        double xError = 200, yError = 200, angleError = 200; //Set arbitrary values that are larger than the MOE

        double previousTime = 0;
        while(timer.milliseconds() <= maxTime && (Math.abs(xError) >= MOE || Math.abs(yError) >= MOE)){
            dt.updatePosition();
            resetCache();

            double k_P = 0.05; //Tune this coefficient
            double currentTime = timer.milliseconds();

            xError = targetX - dt.getX();
            yError = targetY - dt.getY();
            angleError = targetAngle - getAngle();

            double xP = k_P * xError;
            double yP = k_P * yError;
            double anglePow = -0.01 * angleError;



            double xPow = xP;
            double yPow = yP ;


            //driveFieldCentric(drive, strafe, turn);

            driveFieldCentricAuto(xPow, -yPow, targetAngle, speedCap);
            //experiment to see if y is your drive or strafe


            previousTime = currentTime;

        }
    }

    public void moveArm(int target){
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.6);
    }

    public void moveArm(int target, double speed){
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(speed);
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

    public void driveFieldCentricAuto(double drive, double strafe, double angle, double speedCap){
        double heading = Math.toRadians(getAngle());


        double rotVectorX = strafe*Math.cos(-heading) - drive*Math.sin(-heading);
        double rotVectorY = drive*Math.cos(-heading) + strafe*Math.sin(-heading);
        double angleError = normalizeAngle(angle - getAngle());
        double anglePow = -0.01 * angleError;

        rotVectorX *= 1.1;

        double denominator = Math.max(Math.abs(rotVectorY) + Math.abs(rotVectorX) + Math.abs(anglePow), 1);
        double frontLeftPower = (rotVectorY + rotVectorX + anglePow) / denominator;
        double backLeftPower = (rotVectorY - rotVectorX + anglePow) / denominator;
        double frontRightPower = (rotVectorY - rotVectorX - anglePow) / denominator;
        double backRightPower = (rotVectorY + rotVectorX - anglePow) / denominator;

        fLeftMotor.setPower(frontLeftPower * speedCap);
        bLeftMotor.setPower(backLeftPower * speedCap);
        fRightMotor.setPower(frontRightPower * speedCap);
        bRightMotor.setPower(backRightPower * speedCap);
    }

    public void armPurplePixel(){
        pivot.setPosition(0.6);
        arm.setTargetPosition(0);
        arm2.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
        arm2.setPower(0.8);
    }

    public void armStackPick(){
        arm.setTargetPosition(arm.getCurrentPosition() + 150);
        arm2.setTargetPosition(arm2.getCurrentPosition() + 150);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.6);
        arm2.setPower(0.6);
    }

    public void initArmDepo(){
        arm.setTargetPosition(3150);
        arm2.setTargetPosition(3150);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);
        arm2.setPower(0.9);
        pivot.setPosition(0.165);
    }

    public void initArmDepoBlue(){
        arm.setTargetPosition(3150);
        arm2.setTargetPosition(3150);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);
        arm2.setPower(0.9);
        pivot.setPosition(0.16);
    }

    public void zeroArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveFieldCentricAuto(double drive, double strafe, double angle){
        double heading = Math.toRadians(getAngle());


        double rotVectorX = strafe*Math.cos(-heading) - drive*Math.sin(-heading);
        double rotVectorY = drive*Math.cos(-heading) + strafe*Math.sin(-heading);
        double angleError = normalizeAngle(angle - getAngle());
        double anglePow = -0.01 * angleError;

        rotVectorX *= 1.1;

        double denominator = Math.max(Math.abs(rotVectorY) + Math.abs(rotVectorX) + Math.abs(anglePow), 1);
        double frontLeftPower = (rotVectorY + rotVectorX + anglePow) / denominator;
        double backLeftPower = (rotVectorY - rotVectorX + anglePow) / denominator;
        double frontRightPower = (rotVectorY - rotVectorX - anglePow) / denominator;
        double backRightPower = (rotVectorY + rotVectorX - anglePow) / denominator;

        fLeftMotor.setPower(frontLeftPower );
        bLeftMotor.setPower(backLeftPower );
        fRightMotor.setPower(frontRightPower );
        bRightMotor.setPower(backRightPower );
    }

    public void stopDrive(){
        fLeftMotor.setPower(0);
        bLeftMotor.setPower(0);
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
    }

    public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference) {
        // GM0
        double currAngle = getAngle();
        ElapsedTime time = new ElapsedTime();
        while (Math.abs(currAngle - targetAngle) > minDifference
                && time.milliseconds() < timeout
                ) {
            resetCache();
            currAngle = getAngle();
            double angleDiff = Angle.normalize(currAngle - targetAngle);
            double calcP = Range.clip(angleDiff * 0.03, -powerCap, powerCap);
            driveFieldCentricNormal(0, 0, calcP, 1);
        }

        stopDrive();
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

    public void powerArm(int target){
        controller.setPID(0.02, 0, 0.001);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid;
        double cappedPower = Range.clip(power, -1, 1);

        arm.setPower(cappedPower * 0.7);
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
    public void grabLeft(){
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
    }

    public void grabRight(){
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
    }

    public boolean sensePixelLeft(){
        return distance_sensor_right.getDistance((DistanceUnit.CM)) <= 7;
    }

    public boolean sensePixelRight(){
        return distance_sensor_left.getDistance((DistanceUnit.CM)) <= 7;
    }

    //Auto Tools
    public void preparePurpleDrop(){
        pivot.setPosition(0.5);
    }

    public void dropPurplePixel(){
        rightClaw.setPosition(RIGHT_CLAW_OPEN);
    }

    public boolean pressed(GamepadEx driver, GamepadKeys.Button button){
        return driver.wasJustPressed(button);
    }


}

