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

    public int capNumber = 1;

    public CapMech(LinearOpMode op, boolean isAuto) {
        this.isAuto = isAuto;

        armServo = op.hardwareMap.get(Servo.class, "capArm");
        clawServo = op.hardwareMap.get(Servo.class, "capClaw");

        if (!isAuto) home();
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
    public void init () {
        armServo.setPosition(Constants.CAP_INIT);    }

    public void down () {
        armServo.setPosition((capNumber == 1 ? Constants.CAP_DOWN_1 : Constants.CAP_DOWN_2) + downOffset);
    }

    public void up () {
        armServo.setPosition(Constants.CAP_UP + upOffset);
    }

}
