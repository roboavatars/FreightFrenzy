package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//assumes higher levels have higher encoder ticks

@SuppressWarnings("FieldCanBeLocal")
public class Slides {
    private DcMotorEx slidesMotor;

    private double slidesLastTargetPower = 10;
    private int slidesLastTargetPos = -10;


    public Slides(LinearOpMode op, boolean isAuto) {
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "slides");
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        slidesMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("Status", "Slides initialized");
    }

    private void moveSlides(double power, int ticks) {
        if (power != slidesLastTargetPower) {
            slidesMotor.setPower(power);
            slidesLastTargetPower = power;
        }
        if (ticks != slidesLastTargetPos) {
            slidesMotor.setTargetPosition(ticks);
            slidesLastTargetPos = ticks;
        }
    }


    public void resetEncoders () {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void retract() {
        if (slidesMotor.getCurrentPosition()>Constants.SLIDES_RETRACT_TICKS){
            moveSlides(-Constants.SLIDES_POWER,Constants.SLIDES_RETRACT_TICKS);
        } else {
            moveSlides(Constants.SLIDES_POWER,Constants.SLIDES_RETRACT_TICKS);
        }

    }

    public void deposit() {
        if (slidesMotor.getCurrentPosition()>Constants.SLIDES_DEPOSIT_TICKS){
            moveSlides(-Constants.SLIDES_POWER,Constants.SLIDES_DEPOSIT_TICKS);
        } else {
            moveSlides(Constants.SLIDES_POWER,Constants.SLIDES_DEPOSIT_TICKS);
        }

    }

    public void cap() {
        if (slidesMotor.getCurrentPosition()>Constants.SLIDES_CAP_TICKS){
            moveSlides(-Constants.SLIDES_POWER,Constants.SLIDES_CAP_TICKS);
        } else {
            moveSlides(Constants.SLIDES_POWER,Constants.SLIDES_CAP_TICKS);
        }
    }

}
