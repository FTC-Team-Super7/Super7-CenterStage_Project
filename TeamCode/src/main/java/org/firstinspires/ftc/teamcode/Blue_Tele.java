package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="Blue Tele")

public class Blue_Tele extends Base{
    private PIDController controller;

    public static double p =0.02, i=0, d=0.001;
    public static double f = 0;



    private final double ticks_in_degree = 5281 / 360;

    boolean launcherLast = false, launcherCurr = false;
    boolean launched = false;



    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        int target = 0;
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
        boolean incPosLast = false, incPosCurr = false;
        boolean decPosLast = false, decPosCurr = false;
        boolean incPosLastOne = false, incPosCurrOne = false;
        boolean decPosLastOne = false, decPosCurrOne = false;
        boolean clawDown = false;
        boolean oneSet = true;
        boolean test = true;
        boolean test2 = true;
        String armState = "rest";

        int armDepoPos = 2100;
        int restPos = 95;

        double powerCap = 1;

        //Timers
        ElapsedTime autoShutOff = new ElapsedTime();
        ElapsedTime pickUp = new ElapsedTime();

        pivot.setPosition(0.65);   //0.85
        waitForStart();
        while(opModeIsActive()){

            resetCache();



            switchTargetLast = switchTargetCurr;
            switchTargetCurr = gamepad2.y;
            if(switchTargetCurr && !switchTargetLast){
                down = !down;
                if(down){
                    target = restPos;
                    armState = "down";
                }else{
                    target = armDepoPos;
                    armState = "up";
                    oneSet = true;
                }
            }

            incPosLast = incPosCurr;
            incPosCurr = gamepad2.right_stick_button;
            if(incPosCurr && !incPosLast){
                armDepoPos += 100;
                target = armDepoPos;
            }

            decPosLast = decPosCurr;
            decPosCurr = gamepad2.left_stick_button;
            if(decPosCurr && !decPosLast){
                armDepoPos -= 100;
                target = armDepoPos;
            }

            incPosLastOne = incPosCurrOne;
            incPosCurrOne = gamepad2.right_trigger > 0.05;
            if(incPosCurrOne && !incPosLastOne){
                restPos += 100;
                target = restPos;
            }

            decPosLastOne = decPosCurrOne;
            decPosCurrOne = gamepad2.left_trigger > 0.05;
            if(decPosCurrOne && !decPosLastOne){
                restPos -= 100;
                target = restPos;
            }

            if(gamepad2.left_bumper){
                target = 55;
            }






            controller.setPID(0.02, 0, 0.001);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid;
            double cappedPower = Range.clip(power, -1, 1);

            arm.setPower(cappedPower * 0.6);




            if(gamepad1.right_trigger > 0.05){
                powerCap = 0.5;
            }else {
                powerCap = 1;
            }

            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            driveFieldCentric(drive, strafe, turn, powerCap, -90);
            //int armPos = arm.encoderReading();

            if(gamepad2.dpad_right){
                hanger.setPower(-1);
            }else if(gamepad2.dpad_left){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
            }

            if(gamepad2.dpad_up){
                launcher.setPower(-0.4);
            }else if(gamepad2.dpad_down){
                launcher.setPower(0.4);
            }else{
                launcher.setPower(0);
            }


            if(gamepad1.left_trigger > 0.05){
                pivot.setPosition(0.45);
                test2 = true;
                if(test){
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);
                    test = false;

                }

            }

            pivotLast = pivotCurr;
            pivotCurr = !(gamepad1.left_trigger > 0.05);
            if(!pivotLast && pivotCurr){
                pivot.setPosition(0.65);
                test = true;
                if(test2){
                    leftClaw.setPosition(LEFT_CLAW_CLOSE);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                    test2 = false;
                }


            }


            //down 0.45
            //up 0.65
            lastIncre = currIncre;
            currIncre = gamepad1.y || gamepad2.x;

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
            clawAlignCurr = gamepad2.b;
            if(clawAlignCurr && !clawAlignLast){
                clawDown = !clawDown;
                if(clawDown){
                    pivot.setPosition(0.85);
                }else{
                    pivot.setPosition(0.65);
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
            autoPickLast = autoPickCurr;
            autoPickCurr = leftClaw.getPosition() == LEFT_CLAW_CLOSE && rightClaw.getPosition() == RIGHT_CLAW_CLOSE;
            if(autoPickCurr && !autoPickLast){
                pickUp.reset();
                transfer = true;
            }

            if(pickUp.milliseconds() > 200 && transfer){
                pivot.setPosition(0.65);
                up = true;
                transfer = false;


            }











            telemetry.addData("Pos", armPos);
            telemetry.addData("Target", target);
            telemetry.update();
        }


    }
}
