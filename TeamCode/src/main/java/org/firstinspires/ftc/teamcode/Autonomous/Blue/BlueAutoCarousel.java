package org.firstinspires.ftc.teamcode.Autonomous.Blue;

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
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous
public class BlueAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;
    public static double delay = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 9, 41, PI, true, false, true);

        double goToPreloadTime = 2;
        double timeToCarousel = 2;
        double timeToDeposit = 2;
        double timeToPark = 2;
        int autoSteps = 0;

        double reachedSpinPos = -1;
        double startSweepTime = -1;

        double[] preloadScoreCoords;
        double[] spinPose = new double[]{14, 15, 4.4 * PI / 4};
        double[] depositCoords = new double[]{60, 40, 3.5 * PI/2};
        double[] parkCoords = new double[]{36.5, 10, PI};

        robot.carouselAuto = true;

        Path spinPath = null;
        Path depoDuck = null;
        Path gotoP = null;

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            preloadScoreCoords = new double[]{25, 48, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.low;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            preloadScoreCoords = new double[]{20, 48, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.mid;
        } else {
            preloadScoreCoords = new double[]{15, 44, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.high;
        }

        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(robot.x, robot.y, 3 * PI/2, 10, 10, 0, 0),
                new Waypoint(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2] + PI, 2, -10, 0.01, goToPreloadTime),
        };
        Path preloadP = new Path(preloadScoreWaypoints);

        robot.depositingFreight = true;
        robot.depositApproval = false;
        robot.depositState = 0;
        robot.intakeExtendDist = Constants.INTAKE_SLIDES_EXTEND_TICKS/2;

        while (opModeIsActive()) {
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            addPacket("step", autoSteps);

            switch (autoSteps) {
                case 0: //delay
                    if (time.seconds() > delay) {
                        time.reset();
                        autoSteps++;
                        robot.depositState = 3;
                    }
                    break;
                case 1:
                    Pose curPose = preloadP.getRobotPose(Math.min(goToPreloadTime, time.seconds()));
                    robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                    addPacket("robit at deposit pos", robot.isAtPose(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2]));

                    robot.releaseApproval = time.seconds() > goToPreloadTime;//robot.isAtPose(preloadScoreCoords[0], preloadScoreCoords[1], preloadScoreCoords[2]);
//                            && robot.notMoving();

                    if (robot.depositState == 6) {
                        time.reset();
                        Waypoint[] pathToCarousel = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 10, 10, 0, 0),
                                new Waypoint(spinPose[0], spinPose[1], spinPose[2], 1, -5, 0.01, timeToCarousel)
                        };
                        spinPath = new Path(pathToCarousel);

                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 2:
                    Pose curDuck = spinPath.getRobotPose(Math.min(timeToCarousel, time.seconds()));
                    robot.setTargetPoint(new Target(curDuck));

                    if (reachedSpinPos == -1 && time.seconds() > timeToCarousel)//robot.isAtPose(spinPose[0], spinPose[1], spinPose[2], 4, 4, PI/10))
                        reachedSpinPos = time.seconds();
                    addPacket("reachedSpinPos", reachedSpinPos);
                    addPacket("at pos", robot.isAtPose(spinPose[0], spinPose[1], spinPose[2]));

                    robot.carousel.turnon();

                    if (reachedSpinPos != -1) {
                        if (time.seconds() - reachedSpinPos < 10) {
                            robot.carousel.turnon();
                            startSweepTime = time.seconds();
                        } else {
                            robot.carousel.turnoff();
                            robot.intakeApproval = true;

                            //sweep
                            robot.setTargetPoint(20 + 3 * Math.sin(9*(time.seconds() - startSweepTime)), 31 - 18 * Math.sin(2*(time.seconds() - startSweepTime)), -3.5 * PI / 4);

                            if (timeLeft < 10) robot.transferOverride = true;

                            if (robot.intakeState == 3) {
                                robot.intakeApproval = false;
                                autoSteps++;
                            }
                        }
                    }
                    break;
                case 3 :
                    robot.setTargetPoint(20, 10, PI);
                    if (robot.y < 20) {
                        Waypoint[] depositDuck = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta + PI, 10, 10, 0, 0),
                                new Waypoint(40 , 7, 3 * PI/2, 5, 5, 0, 1),
                                new Waypoint(depositCoords[0], depositCoords[1], depositCoords[2] + PI, 10, 10, 0.1, timeToDeposit),
                        };
                        depoDuck = new Path(depositDuck);
                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 4:
                    Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToDeposit));
                    robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                    robot.depositApproval = time.seconds() > 1.5;

                    robot.releaseApproval = time.seconds() > timeToDeposit;//robot.isAtPose(depositCoords[0], depositCoords[1], depositCoords[2], 4, 4, PI/10);
//                            && robot.notMoving();

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
                case 5:
                    robot.setTargetPoint(new Target(gotoP.getRobotPose(Math.min(time.seconds(), timeToPark))));

                    robot.intakeUp = true;
                    robot.capDown = true;

                    if (robot.isAtPose(parkCoords[0], parkCoords[1], parkCoords[2])
                            && robot.notMoving()) {
                        autoSteps++;
                    }
                    break;
                case 6:
                    robot.drivetrain.stop();
                    addPacket("auto done", 0);
                    break;
            }
            robot.update();
        }
        robot.stop();
    }
}

