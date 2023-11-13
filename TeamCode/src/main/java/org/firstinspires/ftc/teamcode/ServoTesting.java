package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo pivot = hardwareMap.get(Servo.class, "pivot");
        pivot.setPosition(0.5);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                pivot.setPosition(pivot.getPosition() +  0.02);
            }else if(gamepad1.dpad_down){
                pivot.setPosition(pivot.getPosition() - 0.02);
            }
        }
    }
}
