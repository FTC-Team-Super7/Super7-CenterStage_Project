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
        ArrayList<Point> path1, path2, initDepositRight, purplePixelRight, initDepositMid, purplePixelMid, initDepositLeft, purplePixelLeft, toStack;
        path1 = new ArrayList<>();
        path1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(135, -42, 0, 0.8),
                                new Point(133, 90, 0, 0.8)

                        )));
        path1 = PathGenerator.interpSplinePath(path1, new Point(86, -51, false, false, 0.8));

        path2 = new ArrayList<>();
        path2.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                // new Point(90, -80, 0, 0.5),
                                new Point( 0, -185, 0, 1),
                                new Point(-120, -215, 0, 1)



                        )));
        path2 = PathGenerator.interpSplinePath(path2, new Point(0, 0, false, false, 1));

        initDepositRight = new ArrayList<>();
        initDepositRight.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(33, -62, 0, 0.5)
                        )
                ));

        initDepositMid = new ArrayList<>();
        initDepositMid.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(50, -62, 0, 0.5)
                        )
                )
        );

        purplePixelRight = new ArrayList<>();
        purplePixelRight.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(86, -42, 0, 0.7)
                        )
                )
        );

        purplePixelMid = new ArrayList<>();
        purplePixelMid.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(95, -26, 0, 0.7)
                        )
                )
        );

        initDepositLeft = new ArrayList<>();
        initDepositLeft.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(67, -62, 0, 0.5)
                        )
                )
        );

        purplePixelLeft = new ArrayList<>();
        purplePixelLeft.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(61, -2, 0, 0.5)
                        )
                )

        );

        toStack = new ArrayList<>();
        toStack.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(120, -47, 0, 0.6),
                                new Point(120, 125, 0, 0.6)
                        )
                )
        );





        initHardware(this);
        resetCache();
        pivot.setPosition(0.8);
        String armState = "rest";
        int target = 0;
        PIDController controller = new PIDController(0.02, 0, 0.001);
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("status", path1.size());

        telemetry.addData("status", "init");
        telemetry.update();
        waitForStart();


        initArmDepo();
//        pivot.setPosition(0.165); //only for middle pos
        sleep(100);
//        ChaseTheCarrotConstantHeading(initDepositMid, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        ChaseTheCarrotConstantHeading(initDepositLeft, 10, 15, 90, 2, 1, 0.05, 0.05,0.03, 0, 0, 5000);
        ChaseTheCarrotConstantHeading(initDepositRight, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
         sleep(500);
        leftClaw.setPosition(LEFT_CLAW_OPEN);
        sleep(500);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        armPurplePixel();
        sleep(750);


//        ChaseTheCarrotConstantHeading(purplePixelLeft, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        ChaseTheCarrotConstantHeading(purplePixelRight, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        ChaseTheCarrotConstantHeading(purplePixelMid, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        sleep(500);
        pivot.setPosition(0.55);
        sleep(500);
        rightClaw.setPosition(RIGHT_CLAW_OPEN);
        sleep(250);
        pivot.setPosition(0.65);
//        zeroArm();
        sleep(300);

        ChaseTheCarrotConstantHeading(toStack, 10, 15, 90, 2, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
        pivot.setPosition(0.55);
        sleep(250);
//        armStackPick();
        sleep(650);
        timer.reset();
        while(timer.milliseconds() < 2000 && distance_sensor_left.getDistance(DistanceUnit.CM) > 25){
            dt.updatePosition();
            resetCache();
            driveFieldCentricAuto(0.2, 0, 90, 1);
            telemetry.addData("Distance", distance_sensor_left.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        stopDrive();

        armStackPick();
        sleep(670);

        timer.reset();
        while(timer.milliseconds()<1000 && distance_sensor_left.getDistance(DistanceUnit.CM) > 13){
            dt.updatePosition();
            resetCache();
            driveFieldCentricAuto(0, -0.2, 90, 1);
        }
        stopDrive();
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        sleep(500);

//        arm.setTargetPosition(180);
//        arm.setTargetPositionTolerance(1);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.5);
//        pivot.setPosition(0.55);
//        ChaseTheCarrotConstantHeading(path1, 10, 15, 90, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        timer.reset();
//        while(timer.milliseconds() < 3000 && distance_sensor_left.getDistance(DistanceUnit.CM) > 8){
//            dt.updatePosition();
//            resetCache();
//            driveFieldCentricAuto(0, -0.3, 90, 1);
//            dt.setX(0);
//            dt.setY(0);
//        }
//        stopDrive();
//        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
//        sleep(500);
//        ChaseTheCarrotConstantHeading(path2, 10, 25, 90, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        arm.setTargetPosition(2500);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.5);
//        pivot.setPosition(0.3);
//        sleep(2500);

//        ChaseTheCarrotConstantHeading(path1, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        sleep(100);
//        ChaseTheCarrotConstantHeading(path2, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        sleep(1000);
//        ChaseTheCarrotConstantHeading(path1, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);
//        sleep(100);
//        ChaseTheCarrotConstantHeading(path2, 10, 15, 0, 1, 1, 0.05, 0.05, 0.03, 0, 0, 5000);


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

