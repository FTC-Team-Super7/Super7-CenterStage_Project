package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTesting extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Motor arm = new Motor(hardwareMap, "arm", true);
        Servo pivot = hardwareMap.get(Servo.class, "pivot");
        arm.resetEncoder(true);
        boolean upLast = false, upCurr = false;
        boolean downLast = false, downCurr = false;
        boolean powLast = false, powCurr = false;
        boolean powDownLast = false, powDownCurr = false;
        boolean increLast = false, increCurr = false;
        double motorPower = 0.3;
        int increment = 20;

        pivot.setPosition(0.78);
        waitForStart();
        while(opModeIsActive()){
            upLast = upCurr;
            upCurr = gamepad1.a;
            if(upCurr && !upLast){
                arm.setTarget(-570);
                arm.toPosition();
                arm.setPower(0.4);
            }

            downLast = downCurr;
            downCurr = gamepad1.b;
            if(downCurr && !downLast){
                arm.setTarget(arm.encoderReading() - increment);
                arm.toPosition();
                arm.setPower(motorPower);
            }

            powLast = powCurr;
            powCurr = gamepad1.dpad_right;
            if(powCurr && !powLast){
                motorPower += 0.1;
            }

            powDownLast = powDownCurr;
            powDownCurr = gamepad1.dpad_left;
            if(powCurr && !powLast){
                motorPower -=0.1;
            }

            increLast = increCurr;
            increCurr = gamepad1.dpad_right;
            if(increCurr && !increLast){
                increment += 10;
            }

            telemetry.addData("Increment", increment);
            telemetry.addData("Power", motorPower);
            telemetry.addData("Pos", arm.encoderReading());
            telemetry.update();


        }
    }
}
