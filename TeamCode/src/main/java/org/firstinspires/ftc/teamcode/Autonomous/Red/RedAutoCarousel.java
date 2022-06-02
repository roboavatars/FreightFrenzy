package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous
public class RedAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 135.0, 41, 0, true, true, true);

        double goToPreloadTime = 1;
        double timeToCarousel = 1;
        double timeToDeposit = 1;
        double timeToPark = 0.75;
        int autoSteps = 1;

        double reachedSpinPos = -1;
        double startSweepTime = -1;

        double[] preloadScoreCoords;
        double[] spinPose = new double[]{130, 15, 7.2 * PI / 4};
        double[] depositCoords = new double[]{128, 44, 11 * PI/6};
        double[] parkCoords = new double[]{112, 16, 3 * PI / 2};

        robot.carouselAuto = true;



        Path preloadDuck = null;
        Path depoDuck = null;
        Path gotoP = null;

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            preloadScoreCoords = new double[]{118, 48, 11 * PI/6};
            robot.cycleHub = Robot.DepositTarget.low;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            preloadScoreCoords = new double[]{118, 48, 11 * PI/6};
            robot.cycleHub = Robot.DepositTarget.mid;
        } else {
            preloadScoreCoords = new double[]{126, 44, 11 * PI/6};
            robot.cycleHub = Robot.DepositTarget.high;
        }

        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(robot.x, robot.y, 3 * PI/2, 10, 10, 0, 0),
                new Waypoint(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2] + PI, 2, -10, 0.01, goToPreloadTime),
        };
        Path preloadP = new Path(preloadScoreWaypoints);

        robot.depositingFreight = true;
        robot.depositApproval = false;
        robot.depositState = 2;

        while (opModeIsActive()) {
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            addPacket("step", autoSteps);

            switch (autoSteps) {
                case 1:
                    Pose curPose = preloadP.getRobotPose(Math.min(goToPreloadTime, time.seconds()));
                    robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                    addPacket("robit at deposit pos", robot.isAtPose(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2]));

                    robot.depositApproval = robot.isAtPose(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2])
                            && robot.notMoving();

                    if (robot.depositState == 6) {
                        time.reset();
                        Waypoint[] pathToCarousel = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 10, 10, 0, 0),
                                new Waypoint(spinPose[0], spinPose[1], spinPose[2], 1, -5, 0.01, timeToCarousel)
                        };
                        preloadDuck = new Path(pathToCarousel);

                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 2:
                    Pose curDuck = preloadDuck.getRobotPose(Math.min(timeToCarousel, time.seconds()));
                    robot.setTargetPoint(new Target(curDuck));

                    if (reachedSpinPos == -1 && robot.isAtPose(spinPose[0], spinPose[1], spinPose[2]))
                        reachedSpinPos = time.seconds();

                    if (reachedSpinPos != -1) {
                        if (time.seconds() - reachedSpinPos < 10) {
                            robot.carousel.turnon();
                            startSweepTime = time.seconds();
                        } else {
                            robot.carousel.turnoff();
                            robot.intakeApproval = true;

                            //sweep
                            robot.setTargetPoint(121, 30 - 5 * Math.sin(3*(time.seconds() - startSweepTime)), 7.5 * PI / 4);

                            if (timeLeft < 5) robot.transferOverride = true;

                            if (robot.intakeState == 3) {
                                robot.intakeApproval = false;
                                Waypoint[] depositDuck = new Waypoint[]{
                                        new Waypoint(robot.x, robot.y, robot.theta + PI, 10, 10, 0, 0),
                                        new Waypoint(depositCoords[0], depositCoords[1], depositCoords[2] + PI, 10, 10, 0, timeToDeposit),
                                };
                                depoDuck = new Path(depositDuck);

                                time.reset();
                                autoSteps++;
                            }
                        }
                    }
                    break;

                case 3:
                    Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToDeposit));
                    robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                    robot.depositApproval = robot.isAtPose(depositCoords[0], depositCoords[1], depositCoords[2])
                            && robot.notMoving();

                    if (robot.depositState == 6) {
                        Waypoint[] goToThePark = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 10, 10, 0, 0),
                                new Waypoint(parkCoords[0], parkCoords[1], parkCoords[2], 1, -5, 0, timeToPark)
                        };
                        gotoP = new Path(goToThePark);

                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 4:
                    robot.setTargetPoint(new Target(gotoP.getRobotPose(Math.min(time.seconds(), timeToPark))));

                    if (robot.isAtPose(parkCoords[0], parkCoords[1], parkCoords[2])
                            && robot.notMoving()) {
                        autoSteps++;
                    }
                    break;
                case 5:
                    robot.drivetrain.stop();
                    addPacket("auto done", 0);
                    break;
            }
            robot.update();
        }
        robot.stop();
    }
}

