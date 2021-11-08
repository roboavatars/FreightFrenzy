package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private DcMotorEx carouselMotor;
    private double lastPower = 0;

    public Carousel(LinearOpMode op) {
        carouselMotor = op.hardwareMap.get(DcMotorEx.class, "carousel");

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setPower(double power) {
        if (power != lastPower) {
            carouselMotor.setPower(power);
            lastPower = power;
        }
    }

    public void rotate() {
        setPower(Constants.CAROUSEL_POWER);
    }

    public void stop() {
        setPower(0);
    }

}
