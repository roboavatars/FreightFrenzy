package org.firstinspires.ftc.teamcode.Autonomous.Blue;

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
@Autonomous (name = "Blue Auto Carousel" , preselectTeleOp = "2 Teleop 2P", group = "Blue")
public class BlueAutoCarousel extends LinearOpMode {
    public BarcodePipeline.Case barcodeCase;
    public static double delay = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 9, 41, PI, true, false, true);

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, false, false);
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
        double[] spinPose = new double[]{16, 16, 4.3 * PI / 4};
        double[] depositCoords = new double[]{63, 28, 3.3 * PI/2};
        double[] parkCoords = new double[]{34.5, 10, PI};

        robot.carouselAuto = true;

        Path spinPath = null;
        Path depoDuck = null;
        Path gotoP = null;

        waitForStart();
        barcodeCase = barcodeDetector.getResult();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            preloadScoreCoords = new double[]{25, 48, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.low;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            preloadScoreCoords = new double[]{21.5, 48, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.mid;
        } else {
            preloadScoreCoords = new double[]{15, 44, 7*PI/6};
            robot.cycleHub = Robot.DepositTarget.high;
        }

        Robot.log("BarcodeCase: " + barcodeCase);
        try {
            barcodeDetector.stop();
        } catch (Exception ignore) {}

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
            addPacket("barcode", barcodeCase);

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
                                new Waypoint(spinPose[0], spinPose[1] + 4, spinPose[2], 1, -5, 0.8, timeToCarousel-0.2),
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
                            if (time.seconds() - reachedSpinPos < 1 || !robot.notMoving()) {
                                robot.drivetrain.setControls(0.3, 0, 0);
                            } else {
                                robot.drivetrain.setControls(0.2, 0, 0);
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
                    robot.setTargetPoint(34, 6, PI/2);
                    if (robot.isAtPose(34, 6, PI/2)
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
                        robot.setTargetPoint(30, 15, PI);
                    } else if (time.seconds() < 4.14) {
                        robot.setTargetPoint(30 - 10 * Math.sin(1 * (time.seconds() - 1)), 15, PI - PI/3 * Math.sin(3 * (time.seconds() - 1)));
                        startSweepTime = time.seconds();
                    } else {
                        robot.setTargetPoint(17 + 4 * Math.sin(5*(time.seconds() - startSweepTime)), 30 - 10 * Math.sin(0.5*(time.seconds() - startSweepTime)), -3.5 * PI / 4);
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
                    if (robot.depositState == 2) autoSteps++;
                    break;
                case 5 :
                    robot.setTargetPoint(20, 6, PI);
                    if (robot.y < 10) {
                        Waypoint[] depositDuck = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta + PI, 5, 5, 0, 0),
                                new Waypoint(40 , 10, 0.5 * PI/2, 5, 5, 0, 1),
                                new Waypoint(depositCoords[0], depositCoords[1], depositCoords[2] + PI, 5, 5, 0.1, timeToDeposit),
                        };
                        depoDuck = new Path(depositDuck);
                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 6:
                    Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToDeposit));
                    robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                    robot.depositApproval = time.seconds() > 1.5;

                    robot.releaseApproval = time.seconds() > timeToDeposit + .5 //robot.isAtPose(depositCoords[0], depositCoords[1], depositCoords[2], 4, 4, PI/10);
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
                case 7:
                    robot.setTargetPoint(new Target(gotoP.getRobotPose(Math.min(time.seconds(), timeToPark))));

                    robot.intakeUp = true;
                    if (time.seconds() > 1) robot.capDown = true;

                    if (time.seconds() > timeToPark) {
                        autoSteps++;
                    }
                    break;
                case 8 :
                    robot.intakeUp = true;
                    robot.capDown = true;
                    robot.setTargetPoint(34, 6, PI/2);
                    if (robot.isAtPose(34, 6, PI/2)
                            && robot.notMoving()) {
                        autoSteps++;
                    }
                    break;
                case 9:
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

