package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private CRServo carouselServo;
    private double lastPower = 0;

    public Carousel(LinearOpMode op) {
        carouselServo = op.hardwareMap.get(CRServo.class, "carousel");

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setPower(double power) {
        if (power != lastPower) {
            carouselServo.setPower(power);
            lastPower = power;
        }
    }

    public void rotate() {
        setPower(1);
    }

    public void stop() {
        setPower(0);
    }

}
