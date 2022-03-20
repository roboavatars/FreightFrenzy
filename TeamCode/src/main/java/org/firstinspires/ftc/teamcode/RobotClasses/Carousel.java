package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        op.telemetry.addData("Status", "Carousel Initialized");
    }

    private void setVelocity(double velocity) {
        if (velocity != lastPower) {
            carouselMotor.setVelocity(velocity);
            lastPower = velocity;
        }
    }

    public double getVelocity() {
        return carouselMotor.getVelocity();
    }
//
//    public void on() {
//        on(Constants.CAROUSEL_VELOCITY);
//    }

    public void on(double velocity) {
        setVelocity(velocity);
    }

    public void stop() {
        setVelocity(0);
    }
}
