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
@Autonomous(name = "0 red old auto", preselectTeleOp = "1 Teleop", group = "Red")
public class WallDepositAuto extends LinearOpMode {

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
        double preloadScoreTime = 1.75;
        double goToWarehouseTime = 0.5;
        double cycleScoreTime = 0.7;
        double parkThreshold = 5;

        // Paths
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;
        double passLineTime = 0;
        int odoDriftAdjustment = 3;

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
                robot.drivetrain.setGlobalControls(0, 0, 0);

                addPacket("path", "initial deposit imo");

                if (robot.slidesInCommand /*|| time.seconds() > preloadScoreTime + 1.5*/) {
                    robot.cycleHub = Robot.DepositTarget.allianceHigh;

                    time.reset();
                    preloadScore = true;
                }
            }

            else if (!goToWarehouse) {

                if (robot.y < 105) {
                    robot.drivetrain.constantStrafeConstant = -0.4;
                    robot.drivetrain.setGlobalControls(0, 0.7, PI/2 - robot.theta > PI/6 ? 0.5 : 0);
                    passLineTime = time.seconds();

                    addPacket("path", "going to warehouse right rn");
                } else if (timeLeft > parkThreshold) {
                    robot.drivetrain.constantStrafeConstant = 0;
                    robot.setTargetPoint(new Target(robot.x, 110 + 3 * (time.seconds() - passLineTime),
                            PI/2 + (0.15 * Math.sin(1.5 * (time.seconds() - passLineTime)))));

                    addPacket("path", "creeping right rn");
                } else {
                    robot.noDeposit = true;
                    robot.setTargetPoint(new Target(140, 112, PI/2));
                    addPacket("path", "going to park right rn");
                }

                if (robot.intakeFull && robot.y >= 110 - odoDriftAdjustment * cycleCounter && robot.x > 132) {

                    robot.depositApproval = true;

                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(140, robot.y, 3*PI/2, 10, 10, 0, 0),
                            new Waypoint(140 + cycleCounter, 86 - odoDriftAdjustment * cycleCounter, 3*PI/2, 5, -5, 0, cycleScoreTime),
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

                robot.drivetrain.constantStrafeConstant = robot.depositingFreight ? -0.1 : -0.5;

                if (Math.abs(PI/2 - robot.theta) > PI/10 && robot.y > 105) {
                    boolean greater = PI/2 - robot.theta < 0;
                    robot.drivetrain.setGlobalControls(0, 0, greater ? -0.6 : 0.6);
                } else {
                    Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                    robot.setTargetPoint(new Target(curPose).theta(PI/2).thetaKp(5).yKp(0.3));
                }

                addPacket("path", "going to deposit right rn");

                if ((robot.depositingFreight || robot.intakeTransfer) && time.seconds() > 6) {
                    robot.cancelAutomation();
                }

                if (time.seconds() > cycleScoreTime && robot.y <= 90 - odoDriftAdjustment * cycleCounter
                        && !robot.intakeTransfer && robot.slidesInCommand) {
                    Robot.log("ctime: " + time.seconds());
                    cycleCounter++;
                    // if (cycleCounter == 2) robot.noExtend = false;

                    goToWarehouse = false;
                    time.reset();
                }
            }

//            else if (!park) {
//                robot.drivetrain.constantStrafeConstant = -0.3;
//
//                robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime, time.seconds())));
//
//                addPacket("path", "parking right rn");
//
//                if (robot.y > 112) {
//                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");
//
//                    park = true;
//                }
//            }

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