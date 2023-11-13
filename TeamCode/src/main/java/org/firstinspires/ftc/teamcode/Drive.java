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
    protected List<LynxModule> allHubs;
    IMU gyro;

    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0, xP = 0, yP = 0;


    public Drive(
            Motor fLeftMotor,
            Motor bLeftMotor,
            Motor fRightMotor,
            Motor bRightMotor,
            IMU imu,
            OpMode m,
            List<LynxModule> allHubs) {

        this.fLeftMotor = fLeftMotor;
        this.fRightMotor = fRightMotor;
        this.bLeftMotor = bLeftMotor;
        this.bRightMotor = bRightMotor;
        this.gyro = imu;
        this.opMode = m;
        this.allHubs = allHubs;
        driveTime.reset();
        xP = 0;
        yP = 0;
    }

    // Very similar to the carrot chasing algo (https://arxiv.org/abs/2012.13227)
    public void ChaseTheCarrot(
            ArrayList<Point> wp,
            int switchTolerance,
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
        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(getX() - wp.get(wp.size() - 1).xP) > error
                || Math.abs(getY() - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout) {
            updatePosition();
            resetCache();
            double x = getX();
            double y = getY();
            double theta = getAngle();

            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                updatePosition();
                resetCache();
                pt++;
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));

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
            xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
            yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
            turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -1, 1);
            yPow = Range.clip(yPow, -1, 1);

            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            driveFieldCentric(-yPow, -turnPow, xPow);
        }
    }

    public void ChaseTheCarrotConstantHeading(ArrayList<Point> wp,
                                              int switchTolerance,
                                              double heading,
                                              double error,
                                              double angleError,
                                              double normalMovementConstant,
                                              double finalMovementConstant,
                                              double turnConstant,
                                              double movementD,
                                              double turnD,
                                              double timeout){
        ChaseTheCarrot(wp, switchTolerance, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }
    public void ChaseTheCarrotSplineHeading(ArrayList<Point> wp,
                                            int switchTolerance,
                                            double error,
                                            double angleError,
                                            double normalMovementConstant,
                                            double finalMovementConstant,
                                            double turnConstant,
                                            double movementD,
                                            double turnD,
                                            double timeout){
        ChaseTheCarrot(wp, switchTolerance, true, false, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }

    public void ChaseTheCarrotSplineHeadingInverted(ArrayList<Point> wp,
                                                    int switchTolerance,
                                                    double error,
                                                    double angleError,
                                                    double normalMovementConstant,
                                                    double finalMovementConstant,
                                                    double turnConstant,
                                                    double movementD,
                                                    double turnD,
                                                    double timeout){
        ChaseTheCarrot(wp, switchTolerance, true, true, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
    }
    public void ChaseTheCarrotCustomHeading(ArrayList<Point> wp,
                                            int switchTolerance,
                                            double error,
                                            double angleError,
                                            double normalMovementConstant,
                                            double finalMovementConstant,
                                            double turnConstant,
                                            double movementD,
                                            double turnD,
                                            double timeout){
        ChaseTheCarrot(wp, switchTolerance, false, false, Double.MAX_VALUE, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout);
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
        yP+=(yV*(driveTime.seconds()-prevTime))/162.15; // <-- Tick to inch conversion factor
        xP+=(xV*(driveTime.seconds()-prevTime))/162.15;
        prevTime = driveTime.seconds();
    }

    public double getAngle() {

//        Orientation orientation = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        return Angle.normalize(Angle.radians_to_degrees(orientation.firstAngle));
        //  return 0;
        return 0;
    }

    public double getX() {
        return xP;
    }

    public double getY() {
        return yP;
    }

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
    public void resetCache() {
        // Clears cache of all hubs
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}