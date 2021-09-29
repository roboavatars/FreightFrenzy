package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Depositor {
    private Servo depositorGateServo;

    private double lastPos = 2;

    public Depositor(LinearOpMode op, boolean isAuto) {
        depositorGateServo = op.hardwareMap.get(Servo.class, "deposit");

        op.telemetry.addData("Status", "depositor initialized");
    }

    private void setPos(double pos){
        if (pos != lastPos) {
            depositorGateServo.setPosition(pos);
            lastPos = pos;
        }
    }

    public void open(){
        setPos(Constants.DEPOSITOR_GATE_OPEN_POS);
    }
    public void close(){
        setPos(Constants.DEPOSITOR_GATE_CLOSE_POS);
    }

}
