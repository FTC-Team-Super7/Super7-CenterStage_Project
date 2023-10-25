package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Servo bucket = hardwareMap.get(Servo.class, "chain");
        //bucket.setPosition(0.1);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                //bucket.setPosition(bucket.getPosition() +  0.02);
            }else if(gamepad1.dpad_down){
                //bucket.setPosition(bucket.getPosition() - 0.02);
            }
        }
    }
}
