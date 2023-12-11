package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public Servo pivot;
    public Servo leftClaw;
    public Servo rightClaw;

    public DistanceSensor right_distance_sensor;
    public DistanceSensor left_distance_sensor;

    public double INTAKE_UP_POS = 0.65, INTAKE_DOWN_POS = 0.45, INTAKE_DEPO_POS  =0.85;

    public Intake(Servo pivot, Servo leftClaw, Servo rightClaw, DistanceSensor right_distance_sensor, DistanceSensor left_distance_sensor){
        this.pivot = pivot;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
        this.right_distance_sensor = right_distance_sensor;
        this.left_distance_sensor = left_distance_sensor;

    }

    public void IntakeUp(){
        pivot.setPosition(INTAKE_UP_POS);
    }

    public void IntakeDown(){
        pivot.setPosition(INTAKE_DOWN_POS);
    }

    public void PrepareDepo(){
        pivot.setPosition(INTAKE_DEPO_POS);
    }


}
