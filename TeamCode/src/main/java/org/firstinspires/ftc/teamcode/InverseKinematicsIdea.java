package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class InverseKinematicsIdea extends LinearOpMode {
    public DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
    int inchesReading;
    int [] rowHeights = {5, 6, 7, 9};
    int armOffset = 11;
    int row = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        int deltaX = inchesReading;
        int deltaY = rowHeights[row - 1] -11;

        double hypotenuse = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double angle = Math.acos(deltaX/hypotenuse);

        //Set cantilever to angle, set hypotenuse to distance
    }
}
