package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Ramsete.VeloPIDController;
import org.firstinspires.ftc.teamcode.Teleop.Teleop;

import com.acmerobotics.dashboard.config.Config;

@Autonomous
@Config
public class MotorPIDTuning extends Teleop {
    public static double Kp = 0.0003;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double target_ticks = 1700;

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

            VeloPIDController left = new VeloPIDController();
            VeloPIDController right = new VeloPIDController();

            ElapsedTime time = new ElapsedTime();

            waitForStart();

            time.reset();

            while(opModeIsActive()){
                if (time.seconds() % 2 < 1){
                    target = target_ticks;
                } else {
                    target = -target_ticks;
                }

                current_right = (motorFrontRight.getVelocity() + motorBackRight.getVelocity())/2;

                right_power = right.output(current_right, target, Kp, Ki, Kd, time.seconds());

                motorBackRight.setPower(right_power);
                motorFrontRight.setPower(right_power);

                telemetry.addData("currentVelo: ", current_right);
                telemetry.addData("target: ", target);
                telemetry.update();
                addPacket("currentVelo", current_right);
                addPacket("target", target);
                sendPacket();


            }


        }
}
