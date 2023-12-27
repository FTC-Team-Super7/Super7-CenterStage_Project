package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="T3_Auto")
public class T3_AutoTest extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Point> path1, path2, initDeposit;
        path1 = new ArrayList<>();
        path1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(65, -65, 0, 0.8),
                                new Point(170, -65, 0, 0.8)

                        )));
        path1 = PathGenerator.interpSplinePath(path1, new Point(0, 0, false, false, 0.8));

        path2 = new ArrayList<>();
        path2.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                // new Point(90, -80, 0, 0.5),
                                new Point(65, -65, 0, 0.8),
                                new Point(0, 0, 0, 0.8)

                        )));
        path2 = PathGenerator.interpSplinePath(path2, new Point(150, -65, false, false, 0.5));

        initDeposit = new ArrayList<>();
        initDeposit.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(35, -60, 0, 0.6)
                        )
                ));


        initHardware(this);
        resetCache();
        pivot.setPosition(0.65);
        String armState = "rest";
        int target = 0;
        PIDController controller = new PIDController(0.02, 0, 0.001);
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("status", path1.size());

        telemetry.addData("status", "init");
        telemetry.update();
        waitForStart();

//        arm.setTargetPosition(1700);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.5);
//        sleep(100);
//        ChaseTheCarrotConstantHeading(initDeposit, 10, 15, 90, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);


        ChaseTheCarrotConstantHeading(path1, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        sleep(100);
        ChaseTheCarrotConstantHeading(path2, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        sleep(1000);
        ChaseTheCarrotConstantHeading(path1, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        sleep(100);
        ChaseTheCarrotConstantHeading(path2, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);


        //
//        telemetry.addData("X", dt.getX());
//        telemetry.addData("Y", dt.getY());
//        telemetry.update();
//        sleep(10000);
        //moveToPosition(30, -65, 0, 2, 2, 5000, 1);
        stopDrive();
//        pivot.setPosition(0.45);
//        rightClaw.setPosition(RIGHT_CLAW_OPEN);
//        arm.setTargetPosition(220);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.5);
//        moveToPosition(170, -65, 0, 2, 2, 5000, 0.8);
//        timer.reset();
//        while(distance_sensor_left.getDistance(DistanceUnit.CM)  > 10){
//            resetCache();
//            dt.updatePosition();
//            telemetry.addData("X: ", dt.getX());
//            telemetry.addData("Y", dt.getY());
//            telemetry.update();
//            driveFieldCentricAuto(0.4, 0, 0, 1);
//        }
//
//        stopDrive();
//
//        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
//        turnTo(0, 3000, 0.5, 2);
//        sleep(500);
//        pivot.setPosition(0.65);
//
//        moveToPosition(60, -65, 0, 2, 2, 5000, 1);
//
//        pivot.setPosition(0.65);
////        arm.setTargetPosition(2500);
////        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        arm.setPower(0.5);
//        //pivot.setPosition(0.25);
//        moveToPosition(0, 10, 0, 2, 2, 5000,  0.8);
//
//
//
//        rightClaw.setPosition(RIGHT_CLAW_OPEN);
//        sleep(500);
////        arm.setTargetPosition(1800);
////        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        arm.setPower(0.15);
//        sleep(1500);


//        sleep(500);
//        moveToPosition(30, -65, 0, 2, 2, 5000);
//        stopDrive();
//        moveToPosition(0, 0, 0, 2, 2, 5000);
//        stopDrive();*/

    }
}

