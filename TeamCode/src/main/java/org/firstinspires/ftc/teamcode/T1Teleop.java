package org.firstinspires.ftc.teamcode;

public class T1Teleop extends Base{

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        boolean launchLast = false, launchCurr = false;
        boolean launched = false;
        double drive, strafe, turn;
        launcher.setPosition(0);
        waitForStart();

        while(opModeIsActive()){

            drive = gamepad1.left_stick_y*-1;
            strafe = gamepad1.right_stick_x;
            turn = gamepad1.right_stick_x;

            driveFieldCentric(drive, strafe, turn);
            if(gamepad1.right_trigger > 0.05){
                sweeper.setPower(1);
            }else if(gamepad1.left_trigger > 0.05){
                sweeper.setPower(-1);
            }else{
                sweeper.setPower(0);
            }

            launchLast = launchCurr;
            launchCurr = gamepad2.options; //Intentionally a Hard Button to Accidentally Press
            if(launchCurr && !launchLast){
                launched = !launched;
                if(launched){
                    launcher.setPosition(0.5);
                }else{
                    launcher.setPosition(0);
                }
            }

            if(gamepad2.dpad_up){
                hanger.setPower(1);
            }else if(gamepad2.dpad_down){
                hanger.setPower(-1);
            }else{
                hanger.setPower(0);
            }







            telemetry.addData("Angle: ", getAngle());
            telemetry.update();




        }
    }
}
