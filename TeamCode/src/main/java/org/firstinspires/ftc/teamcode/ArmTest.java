package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Test")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
        Servo pivot = hardwareMap.get(Servo.class, "pivot");

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setPosition(0.65);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.y){
                arm.setTargetPosition(2000);
                arm2.setTargetPosition(2000);

                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                arm.setPower(1);
                arm2.setPower(1);
            }
        }
    }
}
