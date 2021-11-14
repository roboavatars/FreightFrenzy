package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp(name = "Ramsete Controller Tuning")
@Config
public class RamseteTuning extends LinearOpMode {
    public static double x = 96;
    public static double y = 120;
    public static double theta = 0;
    public static double splineTime = 5;
    public static double b = 0.00032;
    public static double zeta = 0.7;
    public static boolean move = false;
    public static boolean reset = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 54.5, 63, PI/2, false, true);

        waitForStart();

        ElapsedTime time = new ElapsedTime();
        Path path = null;

        while (opModeIsActive()) {
            if (reset) {
                robot.resetOdo(54.5, 63, PI/2);
                reset = false;
            }

            if (!move) {
                path = new Path(new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta, 0, 0, 0, 0),
                        new Waypoint(x, y, theta * PI, 0, 0, 0, splineTime)
                });

                for (int i = 0; i < 25; i++) {
                    Pose pose = path.getRobotPose(splineTime * i/25);
                    drawPoint(pose.x, pose.y, "blue");
                }

                robot.drivetrain.stop();
                time.reset();
            } else if (time.seconds() >= splineTime && robot.isAtPose(x, y, theta * PI)) {
                move = false;
            } else {
                robot.setTargetPoint(path.getRobotPose(Math.min(splineTime, time.seconds())), b, zeta);
            }

            addPacket("Target X", x);
            addPacket("Target Y", y);
            addPacket("Target Theta", theta + " PI");

            robot.update();
        }

        robot.stop();
    }
}
