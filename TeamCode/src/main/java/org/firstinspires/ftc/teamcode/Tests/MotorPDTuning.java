package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Ramsete.PDController;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@Autonomous
@Config
public class MotorPDTuning extends LinearOpMode {
    public static double Kp = 0;
    public static double Kd = 0;
    public static double target_inches = 1000;

    private double right_power;
    private double left_power;
    private double current_left;
    private double current_right;
    private double target;

    @Override
    public void runOpMode() {
        DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        PDController left = new PDController();
        PDController right = new PDController();

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();

        while (opModeIsActive()) {
            if (time.seconds() % 4 < 2) {
                target = target_inches;
            } else {
                target = -target_inches;
            }

            current_left = (motorFrontLeft.getVelocity() + motorBackLeft.getVelocity()) / 2.0;
            current_right = (motorFrontRight.getVelocity() + motorBackRight.getVelocity()) / 2.0;

            left_power = left.output(current_left, target, Kp, Kd, time.seconds());
            right_power = right.output(current_right, target, Kp, Kd, time.seconds());

            motorBackLeft.setPower(left_power);
            motorFrontLeft.setPower(left_power);
            motorBackRight.setPower(right_power);
            motorFrontRight.setPower(right_power);

            telemetry.addData("currentVelo: ", (current_left+current_right)/2);
            telemetry.addData("target: ", target);
            telemetry.update();

            addPacket("currentVelo", (current_left+current_right)/2);
            addPacket("target", target);
            sendPacket();
        }
    }
}
