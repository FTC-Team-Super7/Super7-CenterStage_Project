package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VISION.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Blue Auto")
public class T2_Auto_Blue extends Base {
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
        BluePropThreshold blueThresh = new BluePropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)

                .addProcessor(blueThresh)
                .build();



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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }

            sleep(500);
            pivot.setPosition(0.65);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(750);


        }else if(result.equals("center")){
            timer.reset();
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
            while(timer.milliseconds() < 830){
                resetCache();
                driveFieldCentricAuto(0.4, 0, 0);
            }
            stopDrive();
            turnTo(0, 3000, 0.5, 2);
            stopDrive();
            dropPurplePixel();
            sleep(500);
            pivot.setPosition(0.65);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(500);



            timer.reset();
            while(timer.milliseconds() < 1350){
                resetCache();
                driveFieldCentricAuto(0, -0.4, -90);
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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
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
            sleep(500);

            target = 300;
            armState = "down";
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 450){
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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }

            sleep(500);
            pivot.setPosition(0.65);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            sleep(750);


        }else {
            timer.reset();
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
            while(timer.milliseconds() < 700){
                resetCache();
                driveFieldCentricAuto(0.4, 0, 0);
            }
            stopDrive();

            sleep(500);
            turnTo(-70, 3000, 0.5, 2);
            sleep(500);
            dropPurplePixel();
            sleep(500);
            pivot.setPosition(0.65);
            sleep(250);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            timer.reset();
            while(timer.milliseconds() < 950){
                resetCache();
                driveFieldCentricAuto(0.3, -0.45, -90);
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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }

            timer.reset();
            while(timer.milliseconds() < 1200){
                resetCache();
                driveFieldCentricAuto(0, -0.3, -90);
            }
            stopDrive();

            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(500);
            target = 300;
            armState = "down";

            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 450){
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
                }else if(armState.equals("down") && arm.getCurrentPosition() < 2000){
                    pivot.setPosition(0.65);
                }
            }
            sleep(500);



        }








    }
}
