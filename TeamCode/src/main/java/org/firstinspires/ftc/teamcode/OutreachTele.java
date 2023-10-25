package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BasicDrivingMode")
public class OutreachTele extends LinearOpMode {
    double drive = 0, turn = 0, strafe = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        double drive = 0, turn = 0, strafe = 0;
        double fLeftpow, fRightPow, bLeftPow, bRightPow;
        double bucketExtend = 0.53;
        double bucketRetract = 1;

        double blockClose = 0.65, blockOpen = 0.5;
        boolean blockLast = false, blockCurr = false;
        DcMotor fLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bRightMotor = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor leftArm = hardwareMap.dcMotor.get("verticalLeftSlide");
        DcMotor rightArm = hardwareMap.dcMotor.get("verticalRightSlide");
        Servo bucket = hardwareMap.get(Servo.class, "chain");


        Servo block = hardwareMap.get(Servo.class, "block");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        block.setPosition(blockClose);

        boolean fullTransferLast = false, fullTransferCurr = false;
        boolean liftUp = false;
        boolean liftLast = false, liftCurr = false;
        boolean intakeLast = false, intakeCurr = false;
        boolean intakeUp = true;
        boolean blocked = false;
        boolean isTrue = false;
        boolean dropCurr = false, dropLast = false;
        boolean retractArm = false;
        boolean bucketLast = false, bucketCurr = false;
        boolean bucketDown = true;
        boolean transfer = false;
        boolean clawDrop = false;
        boolean outreachLast = false, outreachCurr = false;
        boolean outtakeOut = false;
        boolean newTransferLast = false, newTransferCurr = false;


        boolean clawLast = false, clawCurr = false;
        boolean clawClosed = true;
        int EXTEND_POS = 1200;
        int RETRACT_POS = -100;

        ElapsedTime timer = new ElapsedTime();

        intakeLeft.setPosition(0.65);
        intakeRight.setPosition(0.35);  // Intake Position

        wrist.setPosition(0.92);
        claw.setPosition(0.05);

        bucket.setPosition(0.1);


        waitForStart();
        while (opModeIsActive()) {


            drive = gamepad1.right_stick_x;
            turn = -1 * gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            isTrue = gamepad1.right_trigger > 0.05;


            fLeftpow = drive + turn - strafe;
            fRightPow = drive - turn - strafe;
            bLeftPow = drive + turn + strafe;
            bRightPow = drive - turn + strafe;

            fLeftMotor.setPower(fLeftpow * 0.3);
            fRightMotor.setPower(fRightPow * 0.3);
            bLeftMotor.setPower(bLeftPow * 0.3);
            bRightMotor.setPower(bRightPow * 0.3);

            liftLast = liftCurr;
            liftCurr = gamepad1.y;
            if (liftCurr && !liftLast) {
                liftUp = !liftUp;
                if (liftUp) {

                    leftArm.setTargetPosition(EXTEND_POS);
                    leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftArm.setPower(1);

                    rightArm.setTargetPosition(EXTEND_POS);
                    rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightArm.setPower(1);

                } else {

                    leftArm.setTargetPosition(RETRACT_POS);
                    leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftArm.setPower(0.7);

                    rightArm.setTargetPosition(RETRACT_POS);
                    rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightArm.setPower(0.7);

                }
            }

            blockLast = blockCurr;
            blockCurr = gamepad1.x;
            if (blockCurr && !blockLast) {
                blocked = !blocked;
                if (blocked) {
                    block.setPosition(blockClose);
                } else {
                    block.setPosition(blockOpen);
                }
            }

            intakeLast = intakeCurr;
            intakeCurr = gamepad1.a;
            if (intakeCurr && !intakeLast) {
                intakeUp = !intakeUp;
                if (intakeUp) {
                    intakeLeft.setPosition(0.65);    //Retract Intake
                    intakeRight.setPosition(0.35);
                    wrist.setPosition(0.92);


                } else {
                    intakeLeft.setPosition(0.16);  //Deploy Intake
                    intakeRight.setPosition(0.84);
                    wrist.setPosition(0.8);

                    clawClosed = false;
                }

            }

            newTransferLast = newTransferCurr;
            newTransferCurr = gamepad1.left_bumper;
            if (newTransferCurr && !newTransferLast) {
                claw.setPosition(0.3);
                clawDrop = true;

            }


            clawLast = clawCurr;
            clawCurr = gamepad1.b;
            if (clawCurr && !clawLast) {
                clawClosed = !clawClosed;
                if (clawClosed) {
                    claw.setPosition(0.05);
                } else {
                    claw.setPosition(0.3);
                }
            }



            outreachLast = outreachCurr;
            if(gamepad1.dpad_up){
                bucket.setPosition(bucket.getPosition() +  0.02);
            }else if(gamepad1.dpad_down){
                bucket.setPosition(bucket.getPosition() - 0.02);
            }

            telemetry.addData("State: ", outtakeOut);
            telemetry.update();

        }
    }
}

