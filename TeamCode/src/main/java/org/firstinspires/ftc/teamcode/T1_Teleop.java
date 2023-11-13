package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class T1_Teleop extends Base {

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
        waitForStart();

        while(opModeIsActive()){
            resetCache();
            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            int armPos = arm.encoderReading();





            driveFieldCentric(drive, strafe, turn, 0.7);
            //driveRobotCentric(drive, strafe, turn, 0.7);

            if(gamepad1.dpad_up){
                hanger.setPower(-1);
            }else if(gamepad1.dpad_down){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
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
            if(pivotCurr && !pivotLast){
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






            /*if(gamepad1.dpad_left){
                pivot.setPosition(pivot.getPosition() - 0.02);
            }else if(gamepad1.dpad_right){
                pivot.setPosition(pivot.getPosition() + 0.02);
            }*/

            if(gamepad2.dpad_up){
                arm.setPower(-0.08);
            }else if(gamepad2.dpad_down){
                arm.setPower(0.08);
            }else{
                arm.setPower(0);
            }

            if(gamepad2.b){
                pivot.setPosition(0.5);
            }









            telemetry.addData("Angle", getAngle());
            telemetry.addData("Arm", armPos);
           // telemetry.addData("Encoder", bLeftMotor.encoderReading());
            telemetry.update();

        }
    }
}
