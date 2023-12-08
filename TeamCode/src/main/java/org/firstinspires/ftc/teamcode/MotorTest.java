package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                arm.setPower(0.75);
            }else if(gamepad1.dpad_down){
                arm.setPower(-0.75);
            }else{
                arm.setPower(0);
            }
        }
    }
}
