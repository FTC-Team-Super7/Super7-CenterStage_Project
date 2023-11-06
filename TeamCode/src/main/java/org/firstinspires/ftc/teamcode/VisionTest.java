package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VISION.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

@TeleOp(name="Cam Test")
public class VisionTest extends LinearOpMode {
    private VisionPortal portal;
    @Override
    public void runOpMode() throws InterruptedException {
        RedPropThreshold redPropThreshold = new RedPropThreshold();

        AprilTagProcessor tags = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setLensIntrinsics(821.993, 821.993, 330.489, 248.997)
                //.setLensIntrinsics(821.993, 821.993, 320, 240)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)

                .addProcessor(tags)
                .build();
        waitForStart();

        while(opModeIsActive()){
            if(tags.getDetections().size() > 0){
                AprilTagDetection tag = tags.getDetections().get(0);

                telemetry.addData("X:", tag.ftcPose.x);
                telemetry.addData("Y: ", tag.ftcPose.y);
                telemetry.addData("Yaw: ", tag.ftcPose.yaw);
            }

            telemetry.update();

        }


    }
}
