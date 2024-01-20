package org.firstinspires.ftc.teamcode;



import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.VISION.BluePropThreshold;
import org.firstinspires.ftc.teamcode.VISION.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;




@Autonomous(name="Red Auto")
@Disabled
public class T2_Auto_Red extends Base {
    int value = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(this);
        resetCache();
        String result = "";

        PIDController controller = new PIDController(0.02, 0, 0.001);
        String armState = "rest";
        int target = 0;
        VisionPortal portal;
        RedPropThreshold redThresh = new RedPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)

                .addProcessor(redThresh)
                .build();



        ElapsedTime timer = new ElapsedTime();
        while(opModeInInit()){
            result = redThresh.getPropPosition();
            telemetry.addData("Pos", result);
            telemetry.update();
        }

        waitForStart();


        if(result.equals("left")){
            timer.reset();
            while(timer.milliseconds() < 400){
                resetCache();
                driveFieldCentricAuto(0.4, -0.25, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();

            sleep(500);

            preparePurpleDrop();
            timer.reset();
            while(timer.milliseconds() < 700){
                resetCache();
                driveFieldCentricAuto(0.4, 0, 0);
            }
            stopDrive();

            sleep(500);
            turnTo(70, 3000, 0.5, 2);
            sleep(500);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(500);
            pivot.setPosition(0.65);
            sleep(250);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            timer.reset();
            while(timer.milliseconds() < 1000){
                resetCache();
                driveFieldCentricAuto(0.2, 0.45, 90);
            }
            stopDrive();

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
            sleep(500);

            timer.reset();
            while(timer.milliseconds() < 1000){
                resetCache();
                driveFieldCentricAuto(0, 0.3, 90);
            }
            stopDrive();

            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            sleep(500);

            target = 300;
            armState = "down";
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 850){
                resetCache();

                driveFieldCentricAuto(-0.4, 0, 90);


                controller.setPID(0.02, 0, 0.001);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

                double power = pid;
                double cappedPower = Range.clip(power, -1, 1);

                arm.setPower(cappedPower * 0.7);

                if(armState.equals("up") && arm.getCurrentPosition() > 1700){
                    pivot.setPosition(0.85);
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }
            sleep(500);






        }else if(result.equals("center")){
            timer.reset();
            while(timer.milliseconds() < 400){
                resetCache();
                driveFieldCentricAuto(0.4, -0.25, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();

            sleep(500);

            preparePurpleDrop();
            timer.reset();
            while(timer.milliseconds() < 830){
                resetCache();
                driveFieldCentricAuto(0.4, 0, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(500);
            pivot.setPosition(0.65);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            sleep(500);

            timer.reset();
            while(timer.milliseconds() < 1250){
                resetCache();
                driveFieldCentricAuto(0, 0.4, 90);
            }
            stopDrive();
            turnTo(90, 3000, 0.5, 2);
            stopDrive();







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
            while(timer.milliseconds() < 875){
                resetCache();
                driveFieldCentricAuto(0, 0.3, 90);
            }
            stopDrive();
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            sleep(500);
//
            target = 300;
            armState = "down";
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 450){
                resetCache();
                driveFieldCentricAuto(-0.4, 0, 90);

                controller.setPID(0.02, 0, 0.001);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

                double power = pid;
                double cappedPower = Range.clip(power, -1, 1);

                arm.setPower(cappedPower * 0.7);

                if(armState.equals("up") && arm.getCurrentPosition() > 1700){
                    pivot.setPosition(0.85);
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }

            sleep(500);
            pivot.setPosition(0.75);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(750);
            turnTo(90, 3000, 0.5, 2);
            sleep(500);


        }else {
            preparePurpleDrop();

            timer.reset();
            while(timer.milliseconds() < 825){
                resetCache();
                driveFieldCentricAuto(0.48, 0.18, 0);
            }
            stopDrive();
            turnTo(0, 2000, 0.5, 2);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(500);
            pivot.setPosition(0.65);
            sleep(500);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);

            timer.reset();
            while(timer.milliseconds() < 1000){
                resetCache();
                driveFieldCentricAuto(-0.1, 0.4, 90);
            }
            stopDrive();
//
//
//
//
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
                driveFieldCentricAuto(0, 0.3, 90);
            }
            stopDrive();
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            sleep(500);





            target = 300;
            armState = "down";
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 450){
                resetCache();
                driveFieldCentricAuto(-0.4, 0, 90);

                controller.setPID(0.02, 0, 0.001);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

                double power = pid;
                double cappedPower = Range.clip(power, -1, 1);

                arm.setPower(cappedPower * 0.7);

                if(armState.equals("up") && arm.getCurrentPosition() > 1700){
                    pivot.setPosition(0.85);
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }

            sleep(500);
            pivot.setPosition(0.65);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(750);

        }








    }
}

