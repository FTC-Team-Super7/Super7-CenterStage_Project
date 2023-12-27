package org.firstinspires.ftc.teamcode.Subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private final Servo rightClaw;
    private final Servo leftClaw;
    private final DistanceSensor leftDistance;
    private final DistanceSensor rightDistance;

    public ClawSubsystem(final HardwareMap hMap, String rightName, String leftName, String rightDistanceName, String leftDistanceName){
        rightClaw = hMap.get(Servo.class, rightName);
        leftClaw = hMap.get(Servo.class, leftName);
        leftDistance = hMap.get(DistanceSensor.class, leftDistanceName);
        rightDistance = hMap.get(DistanceSensor.class, rightDistanceName);
    }

    public void grab(){

    }

    public void release(){

    }


}
