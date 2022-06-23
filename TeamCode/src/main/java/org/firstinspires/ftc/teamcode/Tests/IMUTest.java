package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.Localization.IMU;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@TeleOp(name = "00 IMU Test")
@Disabled
public class IMUTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Drivetrain dt = new Drivetrain(this, 111, 63, PI / 2);
        IMU imu = new IMU(PI / 2, this);

        double prevTime = 0;
        Acceleration acceleration;

        waitForStart();

        while (opModeIsActive()) {
//            addPacket("system status", imu.imu.getSystemStatus());
//            addPacket("calib status", imu.imu.getCalibrationStatus());

            dt.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            imu.updateHeading();
            acceleration = imu.getAccel();


            if (gamepad1.x) {
                imu.resetHeading(PI / 2);
            }

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            addPacket("theta", imu.getTheta());
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            addPacket("y accel", acceleration.yAccel);
            addPacket("x accel", acceleration.xAccel);
            addPacket("z accel", acceleration.zAccel);
            drawDrivetrain(111, 63, imu.getTheta(), "black");
            drawField();
            sendPacket();
        }
    }
}
