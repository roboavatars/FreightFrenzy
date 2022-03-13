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
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous(name = "0 0 Blue Auto Warehouse", preselectTeleOp = "0 Blue Teleop", group = "Blue")
public class BlueAutoWarehouse extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Right;
    public static double strafeConstant = 0.075;
    public static double parkThreshold = 5;
    public static double odoDriftAdjustment = 3;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight to alliance shipping hub
            cycle 5 freight
            park in warehouse
        */

        Robot robot = new Robot(this, 6, 84, PI/2, true, false);
        robot.logger.startLogging(true, false);

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, false);
        barcodeDetector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;
        boolean resetOdo = false;

        // Segment Times
        double cycleScoreTime = 0.7;
        double parkTime = 3;

        // Paths
        Path cycleScorePath = null;

        int cycleCounter = 0;
        double passLineTime = 0;

        waitForStart();

        barcodeCase = barcodeDetector.getResult();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.allianceLow;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.allianceMid;
        } else {
            robot.cycleHub = Robot.DepositTarget.allianceHigh;
        }
        robot.deposit.preload = true;
        Robot.log("Barcode Case: " + barcodeCase);
        try {
            barcodeDetector.stop();
        } catch (Exception ignore) {}
//        detector.setPipeline(Vision.Pipeline.Freight);

        ElapsedTime time = new ElapsedTime();

        robot.intake.flipDown();
        robot.noExtend = true;

        robot.depositingFreight = true;
        robot.depositApproval = true;
        robot.autoFirstCycle = false;

        while (opModeIsActive()) {
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("case", barcodeCase);
            addPacket("time left", timeLeft);
            addPacket("auto stuff", robot.depositApproval + " " +  robot.deposit.armSlidesAtPose() + " " + robot.deposit.armSlidesAtPose());

            if (!preloadScore) {
                robot.drivetrain.setGlobalControls(0, 0, 0);

                addPacket("path", "initial deposit imo");

                if (robot.slidesInCommand) {
                    robot.cycleHub = Robot.DepositTarget.allianceHigh;
                    robot.deposit.preload = false;

                    time.reset();
                    preloadScore = true;
                }
            }

            else if (!goToWarehouse) {
                if (Math.abs(robot.y - (105 - odoDriftAdjustment * cycleCounter)) < 0.5 && !resetOdo) {
                    robot.resetOdo(6, robot.y, PI/2);
                    resetOdo = true;
                }

                if (timeLeft < parkThreshold || cycleCounter == 5) {
                    goToWarehouse = true;
                    cycleScore = true;
                    time.reset();
                } else if (robot.y < 105 - odoDriftAdjustment * cycleCounter) {
                    robot.drivetrain.constantStrafeConstant = strafeConstant;
                    robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI/2 < -PI/10 ? 0.5 : 0);
                    passLineTime = time.seconds();

                    addPacket("path", "going to warehouse right rn");
                } else if (robot.x < 9 || !robot.intakeFull) {
                    robot.drivetrain.constantStrafeConstant = 0;
                    if ((cycleCounter + 1) % 4 < 3) {
                        double y = Math.min(107 + cycleCounter + 5 * (time.seconds() - passLineTime), 121);
                        robot.setTargetPoint(new Target(6, y, PI/2));
                    } else {
                        double x = Math.min(6 + 1 * (time.seconds() - passLineTime) * (time.seconds() - passLineTime), 14);
                        double y = Math.min(107 + cycleCounter + 5 * (time.seconds() - passLineTime), 121) - odoDriftAdjustment * cycleCounter;
                        double theta = Math.max(PI/2 - PI/11 * (time.seconds() - passLineTime), PI/3);
                        robot.setTargetPoint(new Target(x, y, theta));
                    }

                    addPacket("path", "creeping right rn");
                } else if (robot.x >= 9 && robot.intakeFull) {
                    robot.setTargetPoint(4, 108 - odoDriftAdjustment * cycleCounter, PI/2);
                    addPacket("path", "course correcting into wall");
                }

                if (robot.intakeFull && robot.y >= 107 - odoDriftAdjustment * cycleCounter && robot.x < 9) {
                    resetOdo = false;

                    robot.depositApproval = true;

                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(4, robot.y, 3*PI/2, 10, 10, 0, 0),
                            new Waypoint(4 - cycleCounter, 86 - odoDriftAdjustment * cycleCounter, 3*PI/2, 5, -5, 0, cycleScoreTime),
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                }
            }

            else if (!cycleScore) {
                robot.drivetrain.constantStrafeConstant = robot.y > 105 - odoDriftAdjustment * cycleCounter ? strafeConstant : 0;

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(robot.y >= 83 - odoDriftAdjustment * cycleCounter ? PI/2 : curPose.theta + PI));

                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 105) < 0.5 && !resetOdo) {
                    robot.resetOdo(6, robot.y, PI/2);
                    resetOdo = true;
                }

                if ((robot.depositingFreight || robot.intakeTransfer) && time.seconds() > 5) {
                    robot.cancelAutomation();
                }

                if (time.seconds() > cycleScoreTime && robot.y <= 90 - odoDriftAdjustment * cycleCounter && !robot.intakeTransfer && robot.slidesInCommand) {
                    cycleCounter++;
                    robot.autoFirstCycle = false;
                    if (cycleCounter == 2) robot.noExtend = false;

                    resetOdo = false;
                    goToWarehouse = false;
                    time.reset();
                }
            }

            else if (!park) {
                robot.noDeposit = true;
                robot.setTargetPoint(4, 112, PI/2);
                addPacket("path", "going to park right rn");
                if (time.seconds() > parkTime) {
                    park = true;
                }
            }

            else {
                robot.cancelAutomation();
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
