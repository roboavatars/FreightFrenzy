package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private DcMotorEx carouselMotor;
    private double lastPower = 0;
    private double lastPosition = 0;
    public boolean home;

    private boolean isRed;

    public Carousel(LinearOpMode op, boolean isRed) {
        this.isRed = isRed;

        carouselMotor = op.hardwareMap.get(DcMotorEx.class, "carouselMotor");

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setPower(double power) {
        if (power != lastPower) {
            carouselMotor.setPower(power);
            lastPower = power;
        }
    }

    public void on() {
        setPower(Constants.CAROUSEL_POWER);
    }

    public void stop() {
        setPower(0);
    }
}
