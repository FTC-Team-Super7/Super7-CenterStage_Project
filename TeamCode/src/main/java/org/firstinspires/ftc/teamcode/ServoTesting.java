package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoTesting extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
        boolean currIncre = false, lastIncre = false;
        boolean currDecre = false, lastDecre = false;
        Servo launcher = hardwareMap.get(Servo.class, "rightClaw");
        launcher.setPosition(0.2);
        waitForStart();
        while(opModeIsActive()){
            lastIncre = currIncre;
            currIncre = gamepad1.dpad_up;
            if(currIncre && !lastIncre){
                launcher.setPosition(launcher.getPosition() + 0.05);
            }

            lastDecre = currDecre;
            currDecre = gamepad1.dpad_down;
            if(currDecre && !lastDecre){
                launcher.setPosition(launcher.getPosition() - 0.05);
            }

            telemetry.addData("Servo Pos: ", launcher.getPosition());
            telemetry.update();

        }
    }
}
