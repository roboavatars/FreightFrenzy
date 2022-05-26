package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous
public class RedAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 135.0, 41, PI / 2, true, true);

        double goToPreload = 1;
        double timeToCarousel = 1;
        double timeToPark = 0.75;
        int autoSteps = 1;

        double reachedSpinPos = -1;
        double startSweepTime = -1;

        Waypoint[] preload = new Waypoint[]{
                new Waypoint(robot.x, robot.y, PI, 5, 10, 0, 0),
                new Waypoint(118, 48, -7 * PI / 6, 10, 10, 0, 1)
        };
        Path preloadP = new Path(preload);

        Path preloadDuck = null;
        Path depoDuck = null;

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
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            switch (autoSteps) {
                case 1:
                    Pose curPose = preloadP.getRobotPose(Math.min(goToPreload, time.seconds()));
                    robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                    robot.depositApproval = robot.isAtPose(118, 48, -7 * PI / 6, 2, 2, PI / 10)
                            && robot.notMoving();

                    if (robot.depositState == 6) {
                        Waypoint[] pathToCarousel = new Waypoint[]{
                                new Waypoint(118, 48, -PI / 6, 10, 10, 0, 0),
                                new Waypoint(129, 18, 7 * PI / 4, 10, 10, 0, 1)
                        };
                        preloadDuck = new Path(pathToCarousel);

                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 2:
                    Pose curDuck = preloadDuck.getRobotPose(Math.min(timeToCarousel, time.seconds()));
                    robot.setTargetPoint(new Target(curDuck));

                    if (reachedSpinPos == -1 && robot.isAtPose(129, 18, 7 * PI / 4, 2, 2, PI / 10))
                        reachedSpinPos = time.seconds();

                    if (reachedSpinPos != -1) {
                            if (time.seconds() - reachedSpinPos < 5) {
                                robot.carousel.turnon();
                                startSweepTime = time.seconds();
                            } else {
                                robot.carousel.turnoff();
                                robot.intakeApproval = true;

                                //sweep
                                robot.setTargetPoint(115, 20 - 5 * Math.sin(time.seconds() - startSweepTime), 7 * PI / 4);

                                if (robot.intakeState == 3) {
                                    robot.intakeApproval = false;
                                    Waypoint[] depositDuck = new Waypoint[]{
                                            new Waypoint(robot.x, robot.y, 11 * PI / 4, 10, 10, 0, 0),
                                            new Waypoint(118, 48, -7 * PI / 6, 10, 10, 0, 1),
                                    };
                                    depoDuck = new Path(depositDuck);
                                    autoSteps++;
                                }
                            }
                        }
                    break;

                case 3:
                    Pose curDepo = depoDuck.getRobotPose(Math.min(time.seconds(), timeToCarousel));
                    robot.setTargetPoint(new Target(curDepo).theta(curDepo.theta + PI));

                    robot.depositApproval = robot.isAtPose(118, 48, -7 * PI / 6, 2, 2, PI / 10)
                            && robot.notMoving();

                    if (robot.depositState == 6) {
                        time.reset();
                        autoSteps++;
                    }
                    break;
                case 4:
                    Pose curPark = gotoP.getRobotPose(Math.min(time.seconds(), timeToPark));
                    robot.setTargetPoint(new Target(curPark).theta(curPark.theta + PI));

                    if (robot.isAtPose(108, 10, 3 * PI / 2, 2, 2, PI / 10)
                            && robot.notMoving()) {
                        robot.stop();
                    }
                    break;
            }
        }
    }
}

