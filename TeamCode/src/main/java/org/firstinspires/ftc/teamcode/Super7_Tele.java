package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp")
public class Super7_Tele extends Base{
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        double lockedHeading = 0;
        boolean headingLock = false;
        while(opModeIsActive()){
            resetCache();
            double y = gamepad1.left_stick_y * -1;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if(gamepad1.b){ //ofc add button vars here
                headingLock = !headingLock;
                lockedHeading = getAngle();
            }

            double headingDiff = lockedHeading - getAngle();
            double headingCoeff = 0.055 * headingDiff;

            if(headingLock){
                if(turn>0.1){
                    driveFieldCentric(x, y, turn);
                    lockedHeading = getAngle();
                }else{
                    driveFieldCentric(x, y, headingCoeff);
                }
            }else{
                driveFieldCentric(y, x, turn);
            }




        }





    }
}
