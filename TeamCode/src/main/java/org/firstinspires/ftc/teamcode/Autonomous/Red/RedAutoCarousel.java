package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;
import static java.lang.Math.getExponent;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Carousel;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class RedAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 135.0, 41, PI / 2, true, true);

        boolean preloadScore = false;
        boolean goToCarousel = false;
        boolean dropDuck = false;
        boolean goToPark = false;

        double goToPreload = 1;
        double timeToCarousel = 1;
        double timeToPark = 0.75;

        Waypoint[] preload = new Waypoint[]{
                new Waypoint(robot.x, robot.y, PI, 5, 10, 0, 0),
                new Waypoint(118, 48, -7 * PI / 6, 10, 10, 0, 1)
        };
        Path preloadP = new Path(preload);

        Waypoint[] pathToCarousel = new Waypoint[]{
                new Waypoint(118, 48, -PI / 6, 10, 10, 0, 0),
                new Waypoint(129, 18, 7 * PI / 4, 10, 10, 0, 1)
        };
        Path preloadDuck = new Path(pathToCarousel);

        Waypoint[] depositDuck = new Waypoint[]{
                new Waypoint(129, 18, 11 * PI / 4, 10, 10, 0, 0),
                new Waypoint(118, 48, -7 * PI / 6, 10, 10, 0, 1),
        };
        Path depoDuck = new Path(depositDuck);

        Waypoint[] goToThePark = new Waypoint[]{
                new Waypoint(118, 48, -7 * PI / 6, 10, 10, 0, 0),
                new Waypoint(108, 10, 3 * PI / 2, 10, 10, 0, 1)
        };

        Path gotoP = new Path(goToThePark);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.low;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.mid;
        } else {
            robot.cycleHub = Robot.DepositTarget.high;
        }

        robot.depositingFreight = true;
        robot.depositApproval = false;
        robot.depositState = 2;

        while (opModeIsActive()) {
            addPacket("w", robot.w);
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            if (!preloadScore) {
                Pose curPose = preloadP.getRobotPose(Math.min(goToPreload, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                robot.depositApproval = robot.isAtPose(118, 48, -7 * PI / 6, 2, 2, PI / 10)
                        && robot.notMoving();

                if (robot.depositState == 6) {
                    time.reset();
                    preloadScore = true;
                    robot.intakeApproval = true;
                }
            } else if (!goToCarousel) {
                robot.depositApproval = false;

                Pose curDuck = preloadDuck.getRobotPose(Math.min(timeToCarousel, time.seconds()));
                robot.setTargetPoint(new Target(curDuck));

                if (robot.isAtPose(129, 18, 7 * PI / 4, 2, 2, PI/10)) {
                    Carousel samp = new Carousel(this, true);
                    if (time.seconds() < 10) {
                        samp.turnon();
                    } else {
                        samp.turnoff();
                        robot.intakeApproval = true;
                        if (time.seconds() > timeToCarousel + 3) {
                            goToCarousel = true;
                        }
                    }
                }

            } else if (!dropDuck) {
                robot.intakeApproval = false;
                Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToCarousel));
                robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                robot.depositApproval = robot.isAtPose(118, 48, -7 * PI / 6, 2, 2, PI / 10)
                        && robot.notMoving();

                if (robot.depositState == 6) {
                    time.reset();
                    dropDuck = true;
                }
            } else if (!goToPark) {
                Pose curPark = gotoP.getRobotPose(Math.min(time.seconds(), timeToPark));
                robot.setTargetPoint(new Target(curPark).theta(curPark.theta + PI));

                if (robot.isAtPose(108, 10, 3 * PI / 2, 2, 2, PI / 10)
                        && robot.notMoving()) {
                    goToPark = true;
                    robot.stop();
                }
            }
        }
    }
}
