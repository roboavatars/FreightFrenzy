package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Disabled
@Config
@Autonomous(name = "Blue Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Blue")
public class BlueAutoWarehouse extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Right;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 6, 84, PI/2, true, false);
        robot.logger.startLogging(true, true);

//        Vision detector = new Vision(this, true, Vision.Pipeline.Barcode);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;
        boolean resetOdo = false;

        // Segment Times
        double cycleScoreTime = 0.7;
        double parkThreshold = 5;

        // Paths
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;
        double passLineTime = 0;
        int odoDriftAdjustment = 3;

        waitForStart();

//        barcodeCase = detector.getBarcodePipeline().getResult();

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

                if (time.seconds() > 0.5){
                    robot.carousel.out();
                }

                if (robot.slidesInCommand) {
                    robot.cycleHub = Robot.DepositTarget.allianceHigh;

                    time.reset();
                    preloadScore = true;
                }
            }

            else if (!goToWarehouse) {
                if (robot.y < 105) {
                    robot.drivetrain.constantStrafeConstant = 0.3;
                    robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI/2 < -PI/10 ? 0.5 : 0);
                    passLineTime = time.seconds();

                    addPacket("path", "going to warehouse right rn");
                } else if (timeLeft > parkThreshold) {
                    robot.drivetrain.constantStrafeConstant = 0;
                    double y = Math.min(107 + 3 * (time.seconds() - passLineTime), 116);
                    robot.setTargetPoint(new Target(6, y, PI/2));

                    addPacket("path", "creeping right rn");
                } else {
                    robot.noDeposit = true;
                    robot.setTargetPoint(new Target(4, 112, PI/2));
                    addPacket("path", "going to park right rn");
                }

                if (Math.abs(robot.y - 105) < 0.5 && !resetOdo) {
                    robot.resetOdo(6, robot.y, PI/2);
                    resetOdo = true;
                }

                if (robot.intakeFull && robot.y >= 107) {
                    resetOdo = false;

                    robot.depositApproval = true;

                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(4, robot.y, 3*PI/2, 10, 10, 0, 0),
                            new Waypoint(4 + cycleCounter, 86 - odoDriftAdjustment * cycleCounter, 3*PI/2, 5, -5, 0, cycleScoreTime),
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
                robot.drivetrain.constantStrafeConstant = robot.y > 105 ? 0.4 : 0;

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(robot.y >= 83 ? PI/2 : curPose.theta + PI));

                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 105) < 0.5 && !resetOdo) {
                    robot.resetOdo(6, robot.y, PI/2);
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
