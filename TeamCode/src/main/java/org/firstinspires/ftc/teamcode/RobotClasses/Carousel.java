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
    public boolean home;

    private boolean isRed;

    public Carousel(LinearOpMode op, boolean isRed) {
        this.isRed = isRed;

        carousel = op.hardwareMap.get(CRServo.class, "carousel");
        carouselArm = op.hardwareMap.get(Servo.class, "carouselArm");

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setPower(double power) {
        if (power != lastPower) {
            carousel.setPower(power);
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
