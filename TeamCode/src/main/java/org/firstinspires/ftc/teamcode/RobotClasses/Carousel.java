package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel {
    private CRServo carouselServo;

    private double lastPower;

    public Carousel(LinearOpMode op, boolean isAuto) {
        carouselServo = op.hardwareMap.get(CRServo.class, "carousel");

        op.telemetry.addData("Status", "carousel initialized");
    }

    private void setPower(double power){
        if (power != lastPower) {
            carouselServo.setPower(power);
            lastPower = power;
        }
    }

    public void rotate(){
        setPower(1);
    }

    public void stop(){
        setPower(0);
    }

}
