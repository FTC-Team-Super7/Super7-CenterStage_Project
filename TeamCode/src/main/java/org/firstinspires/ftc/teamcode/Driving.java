package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Driving extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            double drive = gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;

            leftDrive.setPower(drive + turn);
            rightDrive.setPower(drive - turn);

        }
    }
}
