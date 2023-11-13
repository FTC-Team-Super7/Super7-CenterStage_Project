package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class HeadingLockTest extends Base{

    boolean headLast = false, headCurr = false;
    boolean lockOn = false;
    double headingCoeff;
    String state = "down";
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        resetYaw();
        Servo launcher = hardwareMap.get(Servo.class, "launch");
        launcher.setPosition(0);
        boolean launchLast = false, launchCurr = false;
        waitForStart();
        double lockedHeading = 0;
        boolean headingLock = false;
        boolean locked = false;
        boolean slidesHighCurr = false, slidesHighLast = false;
        boolean slidesMidCurr = false, slidesMidLast = false;
        boolean slidesLowCurr = false, slidesLowLast = false;
        boolean slidesRestCurr = false, slidesRestLast = false;



        while(opModeIsActive()){
            double y = gamepad1.left_stick_y * -1;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;



            headLast = headCurr;
            headCurr = gamepad1.b;
            if(headCurr && !headLast){ //ofc add button vars here
                headingLock = !headingLock;
                if(headingLock && !locked){
                    lockedHeading = getAngle();
                    locked = true;
                }else{
                    locked = false;
                }

            }
            if(headingLock) {
                double headingDiff = lockedHeading - getAngle();
                headingCoeff = 0.055 * headingDiff;
            }


            if(headingLock){
                if(Math.abs(turn) > 0.1){
                    //driveFieldCentric(x, y, turn);
                    lockedHeading = getAngle();

                }else{
                    //driveFieldCentric(x, y, headingCoeff);
                }
            }else{
                //driveFieldCentric(y, x, turn);
            }

            launchLast = launchCurr;
            launchCurr = gamepad1.options;  //Hard to hit button - Only One Launch!
            if(launchCurr && !launchLast){
                launcher.setPosition(0.5);
            }

            slidesHighLast = slidesHighCurr;
            slidesHighCurr = gamepad2.y;
            if(slidesHighCurr && !slidesHighLast){
                state = "high";
            }

            slidesLowLast = slidesLowCurr;
            slidesLowCurr = gamepad2.x;
            if(slidesLowCurr && !slidesLowLast){
                state = "low";
            }

            slidesMidLast = slidesMidCurr;
            slidesMidCurr = gamepad2.b;
            if(slidesMidCurr && !slidesMidLast){
                state = "mid";
            }

            slidesRestLast = slidesRestCurr;
            slidesRestCurr = gamepad2.a;
            if(slidesRestCurr && !slidesRestLast){
                state = "rest";
            }





            telemetry.addData("Angle: ", getAngle());
            telemetry.addData("Heading Power: ", headingCoeff);
            telemetry.addData("Heading Lock: ", headingLock);
            telemetry.addData("Locked Heading: ", lockedHeading);
            telemetry.update();



        }





    }
}
