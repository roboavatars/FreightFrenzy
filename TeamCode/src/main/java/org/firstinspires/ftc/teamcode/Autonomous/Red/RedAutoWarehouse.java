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
@Autonomous(name = "0 Red Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoWarehouse extends LinearOpMode {

    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 138, 84, PI/2, true, true);
        robot.logger.startLogging(true, true);

//        BarcodeDetector detector = new BarcodeDetector(this, true);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;
        boolean resetOdo = false;

        // Segment Times
        double cycleScoreTime = 1.5;
        double parkThreshold = 5;

        // Paths
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;
        double passLineTime = 0;

        waitForStart();

//        barcodeCase = detector.getResult();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.allianceLow;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.allianceMid;
        } else {
            robot.cycleHub = Robot.DepositTarget.allianceHigh;
        }
//        Robot.log("Barcode Case: " + barcodeCase);

        ElapsedTime time = new ElapsedTime();

        robot.intake.flipDown();
        robot.noExtend = true;

        robot.depositingFreight = true;
        robot.depositApproval = true;

        while (opModeIsActive()) {

            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);
            addPacket("auto stuff", robot.depositApproval + " " +  robot.deposit.armSlidesAtPose() + " " + robot.deposit.armSlidesAtPose());

            if (!preloadScore) {

                addPacket("path", "initial deposit imo");

                if (robot.slidesInCommand) {
                    robot.cycleHub = Robot.DepositTarget.allianceHigh;
                    robot.autoNoTurret = true;

                    time.reset();
                    preloadScore = true;
                }
            }

            else if (!goToWarehouse) {

                if (robot.y < 105 && robot.x < 137 && PI/2 - robot.theta > PI/10) {
                    robot.drivetrain.constantStrafeConstant = -0.4;
                    robot.setTargetPoint(new Target(143, 78, PI/2).xKp(0.55).thetaKp(4));

                    addPacket("path", "going to the wall right rn");
                } else if (robot.y < 105) {
                    robot.drivetrain.constantStrafeConstant = -0.3;

                    robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI/2 > PI/10 ? -0.5 : 0);
                    passLineTime = time.seconds();

                    addPacket("path", "going to warehouse right rn");
                } else if (timeLeft > parkThreshold) {
                    robot.drivetrain.constantStrafeConstant = 0;
                    // double x = 136 + (2 * Math.cos(2 * (time.seconds() - passLineTime)));
                    double y = Math.min(107 + 3 * (time.seconds() - passLineTime), 116);
                    robot.setTargetPoint(new Target(138, y,
                            PI/2 /*+ (0.2 * Math.sin(1.5 * (time.seconds() - passLineTime)))*/));

                    addPacket("path", "creeping right rn");
                } else {
                    robot.noDeposit = true;
                    robot.setTargetPoint(new Target(140, 112, PI/2));
                    addPacket("path", "going to park right rn");
                }

                if (Math.abs(robot.y - 105) < 0.5 && !resetOdo) {
                    robot.resetOdo(138, robot.y, PI/2);
                    resetOdo = true;
                }

                if (robot.intakeFull && robot.y >= 107) {
                    resetOdo = false;

                    robot.depositApproval = true;

                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(140, robot.y, 3*PI/2, 10, 10, 0, 0),
                            new Waypoint(140, 83, 3*PI/2, 5, 1, 0, 0.75),
                            new Waypoint(130, 78.5, 0.70 + PI, 5, 5, 0, cycleScoreTime),
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                } else if (timeLeft < parkThreshold && robot.y > 112) {

                    time.reset();
                    goToWarehouse = true;
                    cycleScore = true;
                }
            }

            else if (!cycleScore) {

                robot.drivetrain.constantStrafeConstant = robot.y > 105 ? -0.7 : 0;

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(robot.y >= 83 ? PI/2 : curPose.theta + PI));

                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 105) < 0.5 && !resetOdo) {
                    robot.resetOdo(138, robot.y, PI/2);
                    resetOdo = true;
                }

                if ((robot.depositingFreight || robot.intakeTransfer) && time.seconds() > 5) {
                    robot.cancelAutomation();
                }

                if (time.seconds() > cycleScoreTime && robot.y <= 90 && !robot.intakeTransfer && robot.slidesInCommand) {
                    cycleCounter++;
                    if (cycleCounter == 2) robot.noExtend = false;

                    resetOdo = false;
                    goToWarehouse = false;
                    time.reset();
                }
            }

            else {
                robot.noDeposit = true;
                robot.drivetrain.stop();
                addPacket("path", "stopped rn");
            }

            robot.update();
        }
        robot.stop();
//        try {
//            detector.stop();
//        } catch (Exception ignore) {}
    }
}
