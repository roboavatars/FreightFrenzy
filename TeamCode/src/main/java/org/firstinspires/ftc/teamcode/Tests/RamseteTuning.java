package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawPoint;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "Ramsete Controller Tuning")
@Config
@Disabled
public class RamseteTuning extends LinearOpMode {
    public static double x = 54.5;
    public static double y = 120;
    public static double theta = 0.5;
    public static double splineTime = 4;
    public static boolean move = false;
    public static boolean reset = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 54.5, 20, PI/2, false, true);

        waitForStart();

        ElapsedTime time = new ElapsedTime();
        Path path = null;
        Pose pose = null;

        while (opModeIsActive()) {
            if (reset || gamepad1.a) {
                robot.resetOdo(54.5, 20, PI/2);
                reset = false;
            }

            if (!move) {
                path = new Path(new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta, 30, 10, 0, 0),
                        new Waypoint(60.5, 70, PI/2, 40, 10, 0, 2),
                        new Waypoint(x, y, theta * PI, 10, -10, 0, splineTime)
                });

                pose = new Pose(robot.x, robot.y, robot.theta, robot.vx, robot.vy, robot.w);

                robot.drivetrain.setControls(-gamepad1.left_stick_y * 0.5, -gamepad1.right_stick_x * 0.5);
                time.reset();
            } else {
                if (time.seconds() >= splineTime) {
                    move = false;
                } else {
                    pose = path.getRobotPose(Math.min(splineTime, time.seconds()));
                    robot.setTargetPoint(pose);
                }
            }

            for (int i = 0; i < 25; i++) {
                Pose pose2 = path.getRobotPose(splineTime * i/25);
                drawPoint(pose2.x, pose2.y, "blue");
            }

            addPacket("Target X", pose.x);
            addPacket("Target Y", pose.y);
            addPacket("Target Theta", pose.theta);
            addPacket("Target VX", pose.vx);
            addPacket("Target VY", pose.vy);
            addPacket("Target W", pose.w);

            robot.update();
        }

        robot.stop();
    }
}
