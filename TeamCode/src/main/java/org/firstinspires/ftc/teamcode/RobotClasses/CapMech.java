package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class CapMech {
    private Servo armServo;
    private Servo clawServo;

    private boolean isAuto;
    public double upOffset = 0;
    public double downOffset = 0;

    public CapMech(LinearOpMode op, boolean isAuto) {
        this.isAuto = isAuto;

        armServo = op.hardwareMap.get(Servo.class, "capArm");
        clawServo = op.hardwareMap.get(Servo.class, "capClaw");

        if (isAuto) armServo.setPosition(Constants.CAP_INIT);
        else home();
        close();

        op.telemetry.addData("Status", "Cap Mech Initialized");
    }

    public void open () {
        clawServo.setPosition(Constants.CAP_HOME);
    }

    public void close () {
        clawServo.setPosition(Constants.CAP_CLOSE);
    }

    public void home () {
        armServo.setPosition(Constants.CAP_HOME);
    }

    public void down () {
        armServo.setPosition(Constants.CAP_DOWN + downOffset);
    }

    public void up () {
        armServo.setPosition(Constants.CAP_UP + upOffset);
    }

}
