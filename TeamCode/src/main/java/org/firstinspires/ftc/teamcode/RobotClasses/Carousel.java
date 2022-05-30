package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private DcMotorEx carouselMotor;
    private CRServoImplEx crServoImplEx1;
    private CRServoImplEx crServoImplEx2;
    private double lastPower = 0;
    private double lastPosition = 0;
    public boolean home;

    private boolean isRed;

    public Carousel(LinearOpMode op, boolean isRed) {
        this.isRed = isRed;

        crServoImplEx1 = op.hardwareMap.get(CRServoImplEx.class, "carousel1");
        if (isRed) {
            crServoImplEx1.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            crServoImplEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        crServoImplEx2 = op.hardwareMap.get(CRServoImplEx.class, "carousel2");
        if (isRed) {
            crServoImplEx2.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            crServoImplEx2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

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

    public void turnon() {
        crServoImplEx1.setPower(1);
        crServoImplEx2.setPower(1);
    }

    public void turnoff() {
        crServoImplEx1.setPower(0);
        crServoImplEx2.setPower(0);
    }

//
//    public void on() {
//        on(Constants.CAROUSEL_VELOCITY);
//    }

}
