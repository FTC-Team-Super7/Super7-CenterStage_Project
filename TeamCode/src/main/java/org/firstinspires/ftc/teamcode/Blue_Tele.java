package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        initHardware(this);
        int highTarget = 2745;
        int prevhighTarget = 2745;
        int [] armHeights = {138, 3070, 2970, 2250};
        double [] pivots = {0.55, 0.19, 0.21, 0.34};
        //2745, 0.24
        int row = 3;
        int lastRow = 3;
        ElapsedTime pivotTime = new ElapsedTime();
        boolean movePivot = false;

        int lowTarget = 50;
        int prevLowTarget = 50;
        int target = 0;
        resetCache();
        controller = new PIDController(0.02, 0, 0.001);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drive dt = new Drive(fLeftMotor, bLeftMotor, fRightMotor, bRightMotor, imu, this);




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
        boolean moveDownLast = false, moveDownCurr = false;

        boolean clawDown = false;
        boolean oneSet = true;
        boolean test = true;
        boolean test2 = true;
        boolean planeAngLast = false, planeAngCurr = false;
        boolean angleToggle = false;
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
            dt.updatePosition();



            switchTargetLast = switchTargetCurr;
            switchTargetCurr = gamepad2.y;
            if(switchTargetCurr && !switchTargetLast){
                down = !down;

                if(down){
                    arm.setTargetPosition(0);
                    arm2.setTargetPosition(0);

                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    arm2.setPower(0.8);
                    pivot.setPosition(0.65);
                    armState = "rest";
                }else{
                    movePivot = true;
                    pivotTime.reset();

                    arm.setTargetPosition(armHeights[row]);
                    arm2.setTargetPosition(armHeights[row]);



                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    arm.setPower(1);
                    arm2.setPower(1);
                    armState = "up";
                }
            }
            //First Row: 3200 0.12, Second: 3070, 0.13, Third: 2970, 0.15

            if(pivotTime.milliseconds() > 500 && movePivot){
                pivot.setPosition(pivots[row]);
                movePivot = false;
            }

            if(gamepad2.dpad_up ){
                row = 0;
            }
            if(gamepad2.dpad_right ){
                row = 1;
            }if(gamepad2.dpad_down ){
                row = 2;
            }if(gamepad2.dpad_left){
                row = 3;
            }

            if(gamepad2.left_bumper){
                leftClaw.setPosition(LEFT_CLAW_OPEN);
            }

            if(gamepad2.right_bumper){
                rightClaw.setPosition(RIGHT_CLAW_OPEN);
            }

            planeAngLast = planeAngCurr;
            planeAngCurr = gamepad1.b;
            if(planeAngCurr && !planeAngLast){
                angleToggle = !angleToggle;
                if(angleToggle){
                    droneAngle.setPosition(0.3);
                }else{
                    droneAngle.setPosition(0.45);
                }
            }

            moveDownLast = moveDownCurr;
            moveDownCurr = gamepad2.a;
            if(moveDownLast && !moveDownCurr){
                arm.setTargetPosition(arm.getCurrentPosition() - 30);
                arm2.setTargetPosition(arm.getCurrentPosition() - 30);

                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                arm.setPower(0.5);
                arm2.setPower(0.5);
            }

//            if(gamepad1.dpad_up){
//                droneAngle.setPosition(droneAngle.getPosition() + 0.02);
//            }if(gamepad1.dpad_down){
//                droneAngle.setPosition(droneAngle.getPosition() - 0.02);
//            }







//
//            if(gamepad2.left_bumper){
//                target = 55;
//            }
//
//
//
//
//
//
//            controller.setPID(0.02, 0, 0.001);
//            int armPos = arm.getCurrentPosition();
//            double pid = controller.calculate(armPos, target);
//            //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
//
//            double power = pid;
//            double cappedPower = Range.clip(power, -1, 1);
//
//            arm.setPower(cappedPower * 0.6);
//
//
//
//
//            if(gamepad1.right_trigger > 0.05){
//                powerCap = 0.5;
//            }else {
//                powerCap = 1;
//            }

            double drive = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double strafe = gamepad1.right_stick_x; // Counteract imperfect strafing
            double turn = gamepad1.left_stick_x;
            driveFieldCentric(drive, strafe, turn, powerCap, -90);
            //int armPos = arm.encoderReading();

            if(gamepad2.right_trigger > 0.05){
                hanger.setPower(-1);
            }else if(gamepad2.left_trigger > 0.05){
                hanger.setPower(1);
            }else{
                hanger.setPower(0);
            }

            if(gamepad1.dpad_up){
                launcher.setPower(-1);
            }else if(gamepad1.dpad_down){
                launcher.setPower(1);
            }else{
                launcher.setPower(0);
            }

//            if(gamepad2.a){
//                arm.setPower(1);
//                arm2.setPower(1);
//            }else if(gamepad2.b) {
//                arm.setPower(-0.8);
//                arm2.setPower(-0.8);
//            }else{
//
//
//                arm.setPower(0);
//                arm2.setPower(0);
//            }




            if(gamepad1.left_trigger > 0.05){
                pivot.setPosition(0.55);
                test2 = true;
                if(test){
                    autoShutOff.reset();
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
            clawAlignCurr = gamepad1.right_bumper;
            if(clawAlignCurr && !clawAlignLast){
                clawDown = !clawDown;
                if(clawDown){
                    pivot.setPosition(0.25);
                }else{
                    pivot.setPosition(0.65);
                }
            }

            if(gamepad1.a){
                leftClaw.setPosition(LEFT_CLAW_CLOSE);
                arm.setTargetPosition(400);
                arm2.setTargetPosition(400);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                arm2.setPower(1);
                pivot.setPosition(0.1);
            }

            if(gamepad2.right_stick_button){
                pivot.setPosition(pivot.getPosition() + 0.02);
            }

            if(gamepad2.left_stick_button){
                pivot.setPosition(pivot.getPosition() - 0.02);
            }



//            if(sensePixelRight()  && autoShutOff.milliseconds() > 1000 && !armState.equals("up")){ //Automatic Pickup of Pixel
//                grabRight();
//                grabbedRight = true;
//                open = false;
//
//
//
//            }
//
//            if(sensePixelLeft() && autoShutOff.milliseconds() > 1000 && !armState.equals("up")){
//                grabLeft();
//                grabbedLeft = true;
//                open = false;
//
//
//
//            }
////
//            autoPickLast = autoPickCurr;
//            autoPickCurr = leftClaw.getPosition() == LEFT_CLAW_CLOSE && rightClaw.getPosition() == RIGHT_CLAW_CLOSE;
//            if(autoPickCurr && !autoPickLast){
//                pickUp.reset();
//                transfer = true;
//            }

            if(pickUp.milliseconds() > 200 && transfer){
                pivot.setPosition(0.65);
                up = true;
                transfer = false;


            }

            if(armState.equals("up") && lastRow != row) {
                arm.setTargetPosition(armHeights[row]);
                arm2.setTargetPosition(armHeights[row]);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                arm2.setPower(0.5);
                pivot.setPosition(pivots[row]);
            }

            lastRow = row;


















            telemetry.addData("Target", target);
            telemetry.addData("X: ", dt.getX());
            telemetry.addData("Y: ", dt.getY());
            telemetry.addData("Angle: ", dt.getAngle());

            telemetry.addData("FLEFT", fLeftMotor.retMotorEx().getVelocity());
            telemetry.addData("FRIGHT", fRightMotor.retMotorEx().getVelocity());
            telemetry.addData("BRIGHT", bRightMotor.retMotorEx().getVelocity());

            telemetry.addData("BLEFT", bLeftMotor.retMotorEx().getVelocity());
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.addData("Arm 2", arm2.getCurrentPosition());
            telemetry.addData("Pivot", pivot.getPosition());
            telemetry.addData("Row", row);
            telemetry.addData("Drone Angle: ", droneAngle.getPosition());
            telemetry.addData("Distance: ", distance_sensor_left.getDistance(DistanceUnit.CM));

            telemetry.update();
        }


    }
}
