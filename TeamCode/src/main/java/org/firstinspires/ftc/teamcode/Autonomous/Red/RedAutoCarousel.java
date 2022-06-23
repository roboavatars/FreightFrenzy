package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodeDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous (name = "Red Auto Carousel" , preselectTeleOp = "2 Teleop 2P", group = "Red")
public class RedAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Right;
    public static double delay = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 135, 41, 0, true, true, true);

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, true, false);
        barcodeDetector.start();

        double goToPreloadTime = 1.5;
        double timeToCarousel = 2;
        double timeToDeposit = 2;
        double timeToPark = 2;
        int autoSteps = 0;
        boolean intookSomthing = false;

        double reachedSpinPos = -1;
        double startSweepTime = -1;

        double[] preloadScoreCoords;
        double[] spinPose = new double[]{129, 16, 7.2 * PI / 4};
        double[] depositCoords = new double[]{95, 31, 5.2 * PI/4};
        double[] parkCoords = new double[]{109, 10, 0};

        robot.carouselAuto = true;

        Path spinPath = null;
        Path depoDuck = null;
        Path gotoP = null;

        waitForStart();
        barcodeCase = barcodeDetector.getResult();
        addPacket("barcode", barcodeCase);

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            preloadScoreCoords = new double[]{118, 48, 11 * PI/6};
            robot.cycleHub = Robot.DepositTarget.low;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            preloadScoreCoords = new double[]{120, 48, 11 * PI/6};
            robot.cycleHub = Robot.DepositTarget.mid;
        } else {
            preloadScoreCoords = new double[]{128, 44, 11 * PI/6};
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
        robot.intakeExtendDist = Constants.INTAKE_SLIDES_HOME_TICKS;

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
                        robot.cycleHub = Robot.DepositTarget.duck;
                        Waypoint[] pathToCarousel = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 10, 10, 0, 0),
                                new Waypoint(spinPose[0], spinPose[1] - 4, spinPose[2], 1, -5, 1, timeToCarousel-0.2),
                                new Waypoint(spinPose[0], spinPose[1], spinPose[2], 1, -5, 0, timeToCarousel)
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
                        if (time.seconds() - reachedSpinPos < 6) {
                            robot.carousel.turnon();
                            if (time.seconds() - reachedSpinPos < 0.1 || !robot.notMoving()) {
                                robot.drivetrain.setControls(0.23, 0, 0);
                            }
                        } else {
                            autoSteps++;
                            time.reset();
                            robot.carousel.turnoff();
                            robot.intakeApproval = true;
                        }
                    }
                    break;
                case 3 :
                    robot.intakeEnabled = false;
                    robot.intakeUp = true;
                    robot.capDown = true;
                    robot.setTargetPoint(parkCoords[0], 6, PI/2);
                    if (robot.isAtPose(parkCoords[0], 6, PI/2)
                            && robot.notMoving()) {
                        autoSteps++;
                    }
                    break;
                case 4:
                    robot.drivetrain.stop();
                    addPacket("auto done", 0);
                    break;
                    /*
                    //sweep
                    if (time.seconds() < 1) {
                        robot.setTargetPoint(114, 15, 0);
                    } else if (time.seconds() < 4.14) {
                        robot.setTargetPoint(114 + 10 * Math.sin(1 * (time.seconds() - 1)), 15,  PI/3 * Math.sin(3 * (time.seconds() - 1)));
                        startSweepTime = time.seconds();
                    } else {
                        robot.setTargetPoint(127 - 4 * Math.sin(5*(time.seconds() - startSweepTime)), 30 - 12 * Math.cos(0.4*(time.seconds() - startSweepTime)), -0.5 * PI / 4);
                    }

                    if (timeLeft < 8) {
                        robot.transferOverride = true;
                    }

                    if (robot.intakeState == 3) {
                        robot.intakeApproval = false;
                        autoSteps++;
                    }
                    break;
                case 4 :
                    robot.setTargetPoint(124, 6, 0);
                    if (robot.y < 10) {
                        Waypoint[] depositDuck = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta + PI, 10, 10, 0, 0),
                                new Waypoint(104 , 10, 1.5 * PI/2, 5, 5, 0, 1),
                                new Waypoint(depositCoords[0], depositCoords[1], depositCoords[2] + PI, 10, 10, 0.1, timeToDeposit),
                        };
                        depoDuck = new Path(depositDuck);
                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 5:
                    Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToDeposit));
                    robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                    robot.depositApproval = time.seconds() > 1.5;

                    robot.releaseApproval = time.seconds() > timeToDeposit//robot.isAtPose(depositCoords[0], depositCoords[1], depositCoords[2], 4, 4, PI/10);
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
                case 6:
                    robot.setTargetPoint(new Target(gotoP.getRobotPose(Math.min(time.seconds(), timeToPark))));

                    robot.intakeUp = true;
                    if (time.seconds() > 1) robot.capDown = true;

                    if (time.seconds() > timeToPark) {
                        autoSteps++;
                    }
                    break;
                case 7 :
                    robot.intakeUp = true;
                    robot.capDown = true;
                    robot.setTargetPoint(parkCoords[0], 6, PI/2);
                    if (robot.isAtPose(parkCoords[0], 6, PI/2)
                            && robot.notMoving()) {
                        autoSteps++;
                    }
                    break;
                case 8:
                    robot.drivetrain.stop();
                    addPacket("auto done", 0);
                    break;
                     */
            }
            robot.update();
        }
        robot.stop();
    }
}

