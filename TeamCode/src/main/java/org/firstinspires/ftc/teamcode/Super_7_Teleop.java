package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class ARM_PIDF extends Base{
    private PIDController controller;

    public static double p =0.02, i=0, d=0.001;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 5281 / 360;

    boolean launcherLast = false, launcherCurr = false;
    boolean launched = false;



    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        resetCache();
        controller = new PIDController(0.02, 0, 0.001);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        //Button Vars
        boolean switchTargetLast = false, switchTargetCurr = false;
        boolean down = true;
        boolean pivotLast = false, pivotCurr = false;
        boolean up = true;
        boolean lastIncre = false, currIncre = false;
        boolean open = false;
        boolean grabbedRight = false, grabbedLeft = false;
        boolean autoPickLast = false, autoPickCurr = false;
        boolean transfer = false;
        boolean clawAlignLast = false, clawAlignCurr = false;
        boolean clawDown = false;

        //Timers
        ElapsedTime autoShutOff = new ElapsedTime();
        ElapsedTime pickUp = new ElapsedTime();

        pivot.setPosition(0.8);
        waitForStart();
        while(opModeIsActive()){

            resetCache();

            switchTargetLast = switchTargetCurr;
            switchTargetCurr = gamepad2.y;
            if(switchTargetCurr && !switchTargetLast){
                down = !down;
                if(down){
                    target = 40;
                }else{
                    target = 2100;
                }
            }

            if(gamepad2.left_bumper){
                target = 15;
            }



            controller.setPID(0.02, 0, 0.001);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid;
            double cappedPower = Range.clip(power, -1, 1);

            arm.setPower(cappedPower * 0.7);

            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            driveFieldCentric(drive, strafe, turn, 0.7);
            //int armPos = arm.encoderReading();

            if(gamepad1.dpad_up){
                hanger.setPower(-1);
            }else if(gamepad1.dpad_down){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
            }

            launcherLast = launcherCurr;
            launcherCurr = gamepad2.a;
            if(launcherCurr && !launcherLast){
                launched = !launched;
                if(launched){
                    launcher.setPosition(LAUNCHER_SHOOT_POS);
                }else{
                    launcher.setPosition(LAUNCHER_INIT_POS);
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

            clawAlignLast = clawAlignCurr;
            clawAlignCurr = gamepad2.x;
            if(clawAlignCurr && !clawAlignLast){
                clawDown = !clawDown;
                if(clawDown){
                    pivot.setPosition(0.98);
                }else{
                    pivot.setPosition(0.8);
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
//
            /*autoPickLast = autoPickCurr;
            autoPickCurr = leftClaw.getPosition() == LEFT_CLAW_CLOSE && rightClaw.getPosition() == RIGHT_CLAW_CLOSE;
            if(autoPickCurr && !autoPickLast){
                pickUp.reset();
                transfer = true;
            }

            if(pickUp.milliseconds() > 200 && transfer){
                pivot.setPosition(0.78);
                up = true;
                transfer = false;


            }*/











            telemetry.addData("Pos", armPos);
            telemetry.addData("Target", target);
            telemetry.update();
        }


    }
}
