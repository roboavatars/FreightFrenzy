package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Ramsete.PDController;

@Autonomous()
    public class MotorPDTuning extends LinearOpMode {
    private double Kp = .6;
    private double Kd = .05;
    private double target_ticks = 1000;

    double right_power;
    double left_power;
    double current_left;
    double current_right;
    double target;

        @Override
        public void runOpMode(){
            DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
            DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
            DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

            PDController left = new PDController();
            PDController right = new PDController();

            ElapsedTime time = new ElapsedTime();

            waitForStart();

            time.reset();

            while(opModeIsActive()){
                if (time.seconds() % 2 < 1){
                    target = target_ticks;
                } else {
                    target = -target_ticks;
                }

                current_left = (motorFrontLeft.getVelocity() + motorBackLeft.getVelocity())/2;
                current_right = (motorFrontRight.getVelocity() + motorBackRight.getVelocity())/2;

                left_power = left.output(current_left, target, Kp, Kd, time.seconds());
                right_power = right.output(current_right, target, Kp, Kd, time.seconds());

                telemetry.addData("currentVelo", (current_left+current_right)/2);
                telemetry.addData("target", target);

                motorBackLeft.setPower(left_power);
                motorFrontLeft.setPower(left_power);
                motorBackRight.setPower(right_power);
                motorFrontRight.setPower(right_power);

                telemetry.update();
            }


        }
}
