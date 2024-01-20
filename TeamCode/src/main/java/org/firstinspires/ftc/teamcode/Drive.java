package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Base;
import org.firstinspires.ftc.teamcode.Angle;
import org.firstinspires.ftc.teamcode.Motor;
import org.firstinspires.ftc.teamcode.Point;

import java.util.ArrayList;
import java.util.List;

public class Drive extends Base {

    protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;
    protected OpMode opMode;
    IMU imu;

    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0, xP = 0, yP = 0;


    public Drive(
            Motor fLeftMotor,
            Motor bLeftMotor,
            Motor fRightMotor,
            Motor bRightMotor,
            IMU imu,
            OpMode m
            ) {

        this.fLeftMotor = fLeftMotor;
        this.fRightMotor = fRightMotor;
        this.bLeftMotor = bLeftMotor;
        this.bRightMotor = bRightMotor;
        this.imu = imu;
        this.opMode = m;

        driveTime.reset();
        xP = 0;
        yP = 0;
    }


    public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference) {
        // GM0
        double currAngle = getAngle();
        ElapsedTime time = new ElapsedTime();
        while (Math.abs(currAngle - targetAngle) > minDifference
                && time.milliseconds() < timeout
                && ((LinearOpMode) opMode).opModeIsActive()) {
            resetCache();
            updatePosition();
            currAngle = getAngle();
            double angleDiff = Angle.normalize(currAngle - targetAngle);
            double calcP = Range.clip(angleDiff * 0.03, -powerCap, powerCap);
            driveFieldCentric(0,calcP,0);
        }

        stopDrive();
    }

    public void updatePosition() {
        // apply mecnaum kinematic model (with wheel velocities [ticks per sec])
        double xV = (fLeftMotor.retMotorEx().getVelocity() + fRightMotor.retMotorEx().getVelocity()
                + bLeftMotor.retMotorEx().getVelocity() + bRightMotor.retMotorEx().getVelocity()) * 0.5;
        double yV =  (-fLeftMotor.retMotorEx().getVelocity() + fRightMotor.retMotorEx().getVelocity()
                + bLeftMotor.retMotorEx().getVelocity() - bRightMotor.retMotorEx().getVelocity()) * 0.5;

        // rotate the vector
        double nx = (xV*Math.cos(Math.toRadians(getAngle())))-(yV*Math.sin(Math.toRadians(getAngle())));
        double nY = (xV*Math.sin(Math.toRadians(getAngle())))+(yV*Math.cos(Math.toRadians(getAngle())));
        xV = nx; yV = nY;

        // integrate velocity over time
        yP+=(yV*(driveTime.seconds()-prevTime)) / 44.8; // <-- Tick to inch conversion factor
        xP+=(xV*(driveTime.seconds()-prevTime)) / 44.8;
        prevTime = driveTime.seconds();
    }

    public double getAngle(){double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angle;}



    public double getX() {
        return xP;
    }

    public double getY() {
        return yP;
    }

    public void setX(double x){xP = x;}

    public void setY(double y){yP=y;}
    public Point getCurrentPosition() {
        return new Point(getX(), getY());
    }


    public void driveFieldCentric(double drive, double turn, double strafe) {
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        double fRightPow, bRightPow, fLeftPow, bLeftPow;
        double botHeading = -Math.toRadians(getAngle());

        //System.out.println(drive + " " + turn + " " + strafe);

        double rotX = drive * Math.cos(botHeading) - strafe * Math.sin(botHeading);
        double rotY = drive * Math.sin(botHeading) + strafe * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        fLeftPow = (rotY + rotX + turn) / denominator;
        bLeftPow = (rotY - rotX + turn) / denominator;
        fRightPow = (rotY - rotX - turn) / denominator;
        bRightPow = (rotY + rotX - turn) / denominator;

        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    }

    public void driveRobotCentric(double drive, double turn, double strafe) {
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code

        double fRightPow = 0, bRightPow = 0, fLeftPow = 0, bLeftPow = 0;

        fLeftPow = -drive + turn - strafe;
        bLeftPow = -drive + turn + strafe;
        fRightPow = drive + turn - strafe;
        bRightPow = drive + turn + strafe;

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];

        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    }

    public void setDrivePowers(double bLeftPow, double fLeftPow, double bRightPow, double fRightPow) {
        bLeftMotor.setPower(bLeftPow);
        fLeftMotor.setPower(fLeftPow);
        bRightMotor.setPower(bRightPow);
        fRightMotor.setPower(fRightPow);
    }

    public void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    public double[] scalePowers(
            double bLeftPow, double fLeftPow, double bRightPow, double fRightPow) {
        double maxPow =
                Math.max(
                        Math.max(Math.abs(fLeftPow), Math.abs(bLeftPow)),
                        Math.max(Math.abs(fRightPow), Math.abs(bRightPow)));
        if (maxPow > 1) {
            fLeftPow /= maxPow;
            bLeftPow /= maxPow;
            fRightPow /= maxPow;
            bRightPow /= maxPow;
        }

        return new double[] {fLeftPow, bLeftPow, fRightPow, bRightPow};
    }

    // Misc. Functions / Overloaded Method Storage

    public double getRobotDistanceFromPoint(Point p2) {
        return Math.sqrt((p2.yP - getY()) * (p2.yP - getY()) + (p2.xP - getX()) * (p2.xP - getX()));
    }

    // BULK-READING FUNCTIONS


    @Override
    public void runOpMode() throws InterruptedException {}
}