package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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

        carouselMotor = op.hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crServoImplEx1 = op.hardwareMap.get(CRServoImplEx.class, "servoOne");
        if (isRed) {
            crServoImplEx1.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            crServoImplEx1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        crServoImplEx2 = op.hardwareMap.get(CRServoImplEx.class, "servoTwo");
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
        crServoImplEx1.setPwmEnable();
        crServoImplEx2.setPwmEnable();
    }

    public void turnoff() {
        crServoImplEx1.setPwmDisable();
        crServoImplEx2.setPwmDisable();
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
