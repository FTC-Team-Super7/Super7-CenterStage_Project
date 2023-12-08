package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.VISION.BluePropThreshold;

@Autonomous
public class T2_Auto extends Base {
    int value = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        resetCache();
        PIDController controller = new PIDController(0.02, 0, 0.001);
        String armState = "rest";
        int target = 0;
        BluePropThreshold blueThresh = new BluePropThreshold();
        String result = "left";


        ElapsedTime timer = new ElapsedTime();
        while(opModeInInit()){
            result = blueThresh.getPropPosition();
            telemetry.addData("Pos", result);
            telemetry.update();
        }

        waitForStart();


        if(result.equals("left")){
            preparePurpleDrop();

            timer.reset();
            while(timer.milliseconds() < 825){
                resetCache();
                driveFieldCentricAuto(0.55, -0.23, 0);
            }
            stopDrive();
            turnTo(0, 2000, 0.5, 2);
            dropPurplePixel();
            sleep(500);
            pivot.setPosition(0.65);
            sleep(500);

            timer.reset();
            while(timer.milliseconds() < 1200){
                resetCache();
                driveFieldCentricAuto(0, -0.4, -90);
            }
            stopDrive();



            /*timer.reset();
            while(timer.milliseconds() < 265){
                resetCache();
                driveFieldCentricAuto(0.3, 0, -90);
            }
            */
            target = 2500;
            armState = "up";

            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 2500){
                resetCache();
                controller.setPID(0.02, 0, 0.001);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

                double power = pid;
                double cappedPower = Range.clip(power, -1, 1);

                arm.setPower(cappedPower * 0.7);

                if(armState.equals("up") && arm.getCurrentPosition() > 1700){
                    pivot.setPosition(0.85);
                }else if(armState.equals("down") && arm.getCurrentPosition() < 1700){
                    pivot.setPosition(0.65);
                }
            }



            timer.reset();
            while(timer.milliseconds() < 650){
                resetCache();
                driveFieldCentricAuto(0, -0.3, -90);
            }
            stopDrive();


            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(800);


            target = 300;
            armState = "down";
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 750){
                resetCache();
                driveFieldCentricAuto(-0.4, 0, -90);

                controller.setPID(0.02, 0, 0.001);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

                double power = pid;
                double cappedPower = Range.clip(power, -1, 1);

                arm.setPower(cappedPower * 0.7);

                if(armState.equals("up") && arm.getCurrentPosition() > 1700){
                    pivot.setPosition(0.85);
                }else if(armState.equals("down") && arm.getCurrentPosition() < 1700){
                    pivot.setPosition(0.65);
                }
            }

            sleep(500);
            pivot.setPosition(0.65);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(750);


        }else if(result.equals("center")){
            while(timer.milliseconds() < 400){
                resetCache();
                driveFieldCentricAuto(0.4, 0.25, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();

            sleep(500);

            preparePurpleDrop();
            timer.reset();
            while(timer.milliseconds() < 780){
                resetCache();
                driveFieldCentricAuto(0.4, 0, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();
            dropPurplePixel();
            sleep(500);
            pivot.setPosition(0.78);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(500);



            timer.reset();
            while(timer.milliseconds() < 1350){
                resetCache();
                driveFieldCentricAuto(-0.1, -0.4, -90);
            }
            stopDrive();
            turnTo(-90, 3000, 0.5, 2);
            stopDrive();


            sleep(250);
            //turnTo(-90, 3000, 0.5, 2);
            //pivot.setPosition(0.4);
            sleep(500);

            timer.reset();
            while(timer.milliseconds() < 650){
                resetCache();
                driveFieldCentricAuto(0, -0.2, -90);
            }
            stopDrive();
            sleep(250);
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            sleep(500);
            //pivot.setPosition(0.6);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        }else {
            preparePurpleDrop();
            timer.reset();
            while(timer.milliseconds() < 400){
                resetCache();
                driveFieldCentricAuto(0.4, 0.25, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();

            timer.reset();
            while(timer.milliseconds() < 800){
                resetCache();
                driveFieldCentricAuto(0.3, 0, -30);
            }
            stopDrive();
            turnTo(-30, 3000, 0.5, 2);
            stopDrive();

            sleep(500);
            while(timer.milliseconds() < 1000){
                resetCache();
                driveFieldCentricAuto(0.3, 0.3, -40);
            }
            stopDrive();

        }

//        timer.reset();
//        while(timer.milliseconds() < 2500){
//            resetCache();
//            driveFieldCentricAuto(0, 0.45, -90);
//            /*if(timer.milliseconds() < 300){
//                arm.setPower(0.15);
//            }*/
//        }
//        stopDrive();
//
//
//
//
////
//
//
//        turnTo(-90, 3000, 0.5, 2);
//        stopDrive();
//        sleep(500);
//        pivot.setPosition(0.65);
//        rightClaw.setPosition(RIGHT_CLAW_OPEN);
//        sleep(500);
//
//        timer.reset();
//        while(distance_sensor_right.getDistance(DistanceUnit.CM) > 10){
//            resetCache();
//            driveFieldCentricAuto(0, 0.25, -90);
//        }
//        stopDrive();
//        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
//        sleep(500);
////        timer.reset();
////        while(timer.milliseconds() < 300){
////            resetCache();
////            driveFieldCentricAuto(-0.1, 0.15, -90);
////
////        }
////        sleep(350);
//
//        timer.reset();
//        while(timer.milliseconds() < 2850){
//            resetCache();
//            driveFieldCentricAuto(0, -0.5, -90);
//        }
//        stopDrive();
//        rightClaw.setPosition(RIGHT_CLAW_OPEN);
//        sleep(1000);
//        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
//        pivot.setPosition(0.8);
//
//        timer.reset();
//        timer.reset();
//        while(timer.milliseconds() < 2500){
//            resetCache();
//            driveFieldCentricAuto(0, 0.45, -90);
//            /*if(timer.milliseconds() < 300){
//                arm.setPower(0.15);
//            }*/
//        }
//        stopDrive();
//        turnTo(-90, 3000, 0.5, 2);
//        stopDrive();
//        sleep(500);
//        pivot.setPosition(0.65);
//        rightClaw.setPosition(RIGHT_CLAW_OPEN);
//        sleep(500);
//
//        timer.reset();
//        while(distance_sensor_right.getDistance(DistanceUnit.CM) > 10){
//            resetCache();
//            driveFieldCentricAuto(0, 0.25, -90);
//        }
//        stopDrive();
//        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
//        sleep(350);
//
//        timer.reset();
//        while(timer.milliseconds() < 2850){
//            resetCache();
//            driveFieldCentricAuto(0, -0.5, -90);
//        }
//        stopDrive();
//        sleep(500);






    }
}
