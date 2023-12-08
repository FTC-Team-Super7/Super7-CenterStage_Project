package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class T2_Teleop extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        resetCache();

        //Button Variables
        boolean launcherLast = false, launcherCurr = false;
        boolean launched = false;
        boolean pivotLast = false, pivotCurr = false;
        boolean up = true;
        boolean dropLast = false, dropCurr = false;
        boolean upLast = false, upCurr = false;
        boolean incLast = false, incCurr = false;
        boolean decLast = false, decCurr = false;
        boolean currIncre = false, lastIncre = false;
        boolean open = true;
        boolean autoLock = true;
        double drivePow = 0.7;


        boolean shutOff = false;

        //arm.resetEncoder(true);
        waitForStart();

        while(opModeIsActive()){


            resetCache();
            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            //int armPos = arm.encoderReading();





            driveFieldCentric(drive, strafe, turn, drivePow);

            //driveRobotCentric(drive, strafe, turn, 0.7);

            if(gamepad1.dpad_up){
                hanger.setPower(-1);
            }else if(gamepad1.dpad_down){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
            }

            incLast = incCurr;
            incCurr = gamepad2.dpad_right;
            if(incCurr && !incLast){
                drivePow += 0.1;
            }

            decLast = decCurr;
            decCurr = gamepad2.dpad_left;
            if(decCurr && !decLast){
                drivePow -= 0.1;
            }



            launcherLast = launcherCurr;
            launcherCurr = gamepad2.a;
            if(launcherCurr && !launcherLast){
                launched = !launched;
                if(launched){
                    launcher.setPosition(LAUNCHER_SHOOT_POS);
                }else{
                    launcher.setPosition(LAUNCHER_INIT_POS);
                }
            }


            pivotLast = pivotCurr;
            pivotCurr = gamepad1.a;
            if(pivotCurr && !pivotLast && !gamepad1.start){
                up = !up;
                if(up){
                    pivot.setPosition(BOX_MID_POS);
                }else{
                    pivot.setPosition(BOX_INTAKE_POS);
                }
            }

            dropLast = dropCurr;
            dropCurr = gamepad2.x;
            if(dropCurr && !dropLast){
                pivot.setPosition(pivot.getPosition() - 0.04);
            }

            upLast = upCurr;
            upCurr = gamepad2.y;
            if(upCurr && !upLast){
                pivot.setPosition(pivot.getPosition() + 0.04);
            }






            if(gamepad1.dpad_left){
                pivot.setPosition(pivot.getPosition() - 0.02);
            }else if(gamepad1.dpad_right){
                pivot.setPosition(pivot.getPosition() + 0.02);
            }

            if(gamepad2.dpad_up){
                arm.setPower(-0.65);
            }else if(gamepad2.dpad_down){
                arm.setPower(0.15);
            }else{
                arm.setPower(0);
            }

            if(gamepad2.b && !gamepad2.start){
                pivot.setPosition(0.5);
            }

            lastIncre = currIncre;
            currIncre = gamepad1.y;
            autoLock = !(currIncre && !lastIncre);
            if(currIncre && !lastIncre){
                open = !open;
                if(open){
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);
                }else{
                    leftClaw.setPosition(LEFT_CLAW_CLOSE);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                }
            }

            if(sensePixel() && autoLock){ //Automatic Pickup of Pixel
                autoGrab();
            }













            telemetry.addData("Angle", getAngle());
            telemetry.addData("Arm", arm.encoderReading());

            telemetry.addData("Drive Power", drivePow);
            telemetry.addData("Pivot Pos", pivot.getPosition());
            telemetry.addData("Distance", distance_sensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }
}
