package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class T2_Teleop extends Base {
    DcMotorEx arm;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        resetCache();


        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Button Variables
        boolean launcherLast = false, launcherCurr = false;
        boolean launched = false;
        boolean pivotLast = false, pivotCurr = false;
        boolean up = true;
        boolean dropLast = false, dropCurr = false;
        boolean upLast = false, upCurr = false;
        boolean incLast = false, incCurr = false;
        boolean decLast = false, decCurr = false;
        boolean currIncre = false, lastIncre = false;
        boolean open = false;
        boolean transfer = false;
        boolean autoLock = true;
        boolean autoPickCurr = false, autoPickLast = false;
        boolean auto;
        double drivePow = 0.7;
        double target = 0;
        double p = 0.02, i = 0, d = 0.001;
        double ticks_in_degree = 5281/360;
        boolean switchTargetLast = false, switchTargetCurr = false;
        boolean down = true;
        PIDController controller = new PIDController(p, i, d);
        ElapsedTime autoShutOff = new ElapsedTime();
        ElapsedTime matchTime = new ElapsedTime();
        ElapsedTime pickUp = new ElapsedTime();
        boolean grabbedRight = false, grabbedLeft = false;
        boolean firstTime = false;

        boolean shutOff = false;

        //arm.resetEncoder(true);
        waitForStart();

        while(opModeIsActive()){


            resetCache();
            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            //int armPos = arm.encoderReading();








            driveFieldCentric(drive, strafe, turn, drivePow);

            switchTargetLast = switchTargetCurr;
            switchTargetCurr = gamepad2.y;
            if(switchTargetCurr && !switchTargetLast){
                down = !down;
                if(down){
                    target = 50;
                }else{
                    target = 800;
                }
            }

            controller.setPID(0.02, 0, 0.001);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid;
            double cappedPower = Range.clip(power, -1, 1);

            arm.setPower(cappedPower * 0.7);

            //driveRobotCentric(drive, strafe, turn, 0.7);

            if(gamepad1.dpad_up){
                hanger.setPower(-1);
            }else if(gamepad1.dpad_down){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
            }





            incLast = incCurr;
            incCurr = gamepad2.dpad_right;
            if(incCurr && !incLast){
                drivePow += 0.1;
            }

            decLast = decCurr;
            decCurr = gamepad2.dpad_left;
            if(decCurr && !decLast){
                drivePow -= 0.1;
            }



            launcherLast = launcherCurr;
            launcherCurr = gamepad2.a;
            if(launcherCurr && !launcherLast){
                launched = !launched;
                if(launched){

                }else{

                }
            }


            pivotLast = pivotCurr;
            pivotCurr = gamepad1.a;
            if(pivotCurr && !pivotLast && !gamepad1.start){
                up = !up;
                if(up){
                    pivot.setPosition(0.8);
                }else{
                    pivot.setPosition(0.65);
                }
            }








            if(gamepad1.dpad_left){
                pivot.setPosition(pivot.getPosition() - 0.02);
            }else if(gamepad1.dpad_right){
                pivot.setPosition(pivot.getPosition() + 0.02);
            }

            if(gamepad2.dpad_up){
                arm.setPower(-0.65);
            }else if(gamepad2.dpad_down){
                arm.setPower(0.15);
            }else{
                arm.setPower(0);
            }

            if(gamepad2.b && !gamepad2.start){
                pivot.setPosition(0.5);
            }

            lastIncre = currIncre;
            currIncre = gamepad1.y;

            if(currIncre && !lastIncre){
                open = !open;
                if(open){
                    autoShutOff.reset();
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);
                }else{
                    leftClaw.setPosition(LEFT_CLAW_CLOSE);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                }
            }

            if(sensePixelRight()  && autoShutOff.milliseconds() > 1000){ //Automatic Pickup of Pixel
                grabRight();
                grabbedRight = true;
                open = false;



            }

            if(sensePixelLeft() && autoShutOff.milliseconds() > 1000){
                grabLeft();
                grabbedLeft = true;
                open = false;



            }

            autoPickLast = autoPickCurr;
            autoPickCurr = leftClaw.getPosition() == LEFT_CLAW_CLOSE && rightClaw.getPosition() == RIGHT_CLAW_CLOSE && distance_sensor_left.getDistance(DistanceUnit.CM) < 5 && distance_sensor_right.getDistance(DistanceUnit.CM) < 5;
            if(autoPickCurr && !autoPickLast){
                pickUp.reset();
                transfer = true;
            }

            if(pickUp.milliseconds() > 200 && transfer){
                pivot.setPosition(0.78);
                up = true;
                transfer = false;


            }


















            telemetry.addData("Angle", getAngle());
            telemetry.addData("Arm", arm.getCurrentPosition());

            telemetry.addData("Drive Power", drivePow);
            telemetry.addData("Pivot Pos", pivot.getPosition());
            telemetry.addData("Distance", distance_sensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Left", distance_sensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Grab Left", grabbedLeft);
            telemetry.addData("Grab Right", grabbedRight);
            telemetry.update();

        }
    }
}
