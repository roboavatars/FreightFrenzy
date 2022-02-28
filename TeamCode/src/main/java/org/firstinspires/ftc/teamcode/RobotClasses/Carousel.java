package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private CRServo carousel;
    private Servo carouselArm;
    private double lastPower = 0;
    private double lastPosition = 0;
    private boolean isRed;
    public boolean isOut = false;

    public Carousel(LinearOpMode op, boolean isRed) {
        this.isRed = isRed;

        carousel = op.hardwareMap.get(CRServo.class, "carousel");
        carouselArm = op.hardwareMap.get(Servo.class, "carouselArm");

        home();

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setPower(double power) {
        if (power != lastPower) {
            carousel.setPower(power);
            lastPower = power;
        }
    }

    public void on() {
        setPower(Constants.CAROUSEL_POWER * (isRed ? 1 : -1));
    }

    public void stop() {
        setPower(0);
    }

    private void setPosition(double position) {
        if (position != lastPosition) {
            carouselArm.setPosition(position);
            lastPosition = position;
        }
    }

    public void home() {
        isOut = false;
        setPosition(Constants.CAROUSEL_HOME);
    }

    public void out() {
        isOut = true;
        setPosition(Constants.CAROUSEL_OUT);
    }
}
