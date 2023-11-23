package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setPosition(0);
        waitForStart();
        while(opModeIsActive()){
            launcher.setPosition(0.5);
        }
    }
}
