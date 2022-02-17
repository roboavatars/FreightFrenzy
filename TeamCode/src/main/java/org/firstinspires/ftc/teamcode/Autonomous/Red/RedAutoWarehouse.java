package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous(name = "0 0 Red Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoWarehouse extends LinearOpMode {
    public static int barcodeCase = 1; // 0 = left, 1 = mid, 2 = right

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 138, 81, PI/2, true, true);
        robot.logger.startLogging(true, true);

//        BarcodeDetector detector = new BarcodeDetector(this, true);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;

        // Segment Times
        double preloadScoreTime = 1.75;
        double goToWarehouseTime = 0.5;
        double cycleScoreTime = 0.7;
        double parkTime = 0.5;

//        double cycleX = 138;
//        double depositX = 117;
//        double depositY = 68;
//        double depositTh = PI/10;

//        double[][] preloadScoreCoord = {{111.5, 63, 5*PI/6}, {115.5, 63, 5*PI/6}, {119, 63, 5*PI/6}};

        // Paths
        Path goToWarehousePath = null;
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;

        waitForStart();

        /*
        if(detector.getResult() == BarcodePipeline.Case.Left){
            barcodeCase = 0;
        } else if(detector.getResult() == BarcodePipeline.Case.Middle){
            barcodeCase = 1;
        } else {
            barcodeCase = 2;
        }
        Robot.log("Barcode Case: " + barcodeCase);
         */

        if (barcodeCase == 0) {
            robot.cycleHub = Robot.DepositTarget.allianceLow;
        } else if (barcodeCase == 1) {
            robot.cycleHub = Robot.DepositTarget.allianceMid;
        } else {
            robot.cycleHub = Robot.DepositTarget.allianceHigh;
        }

        Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                new Waypoint(robot.x, robot.y, PI/2/*robot.theta*/, 10, 2,0, 0),
                new Waypoint(140,111,PI/2,10,-2,0, goToWarehouseTime)
        };
        goToWarehousePath = new Path(goToWarehouseWaypoints);

        ElapsedTime time = new ElapsedTime();

        robot.intake.flipDown();
        robot.noExtend = true;

        robot.depositingFreight = true;
        robot.depositApproval = true;

        while (opModeIsActive()) {
            if (!preloadScore) {
                if (!robot.depositingFreight/*time.seconds() > preloadScoreTime + 1.5*/) {
                    time.reset();
                    preloadScore = true;
                    robot.cycleHub = Robot.DepositTarget.allianceHigh;
                }
            } else if (!goToWarehouse) {
                if (!robot.intakeFull && robot.y >= 110 && time.seconds() > goToWarehouseTime + 0.1) {
                    robot.setTargetPoint(137, 111 + 2.5 * (time.seconds() - goToWarehouseTime),
                            PI/2 + 0.15 * Math.sin(2 * (time.seconds() - goToWarehouseTime)));
                }
//                else if (Math.abs(robot.theta - PI/2) > PI/8 || robot.x < 137) {
//                    robot.setTargetPoint(new Target(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime, time.seconds()))))/*.yKp(0.25).thetaKp(.4))*/;
//                }
                else {
                    Pose curPose = goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime, time.seconds()));
                    robot.setTargetPoint(new Target(curPose).theta(PI/2).thetaKp(3.5));
                }

                addPacket("path", "going to warehouse right rn");

                if (robot.intakeFull/*time.seconds() > goToWarehouseTime + 0.5*/) {
                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(140, robot.y, 3*PI/2, 10, 10, 0, 0),
                            new Waypoint(140, 86, 3*PI/2, 10, -5, 0, cycleScoreTime),
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                }
            } else if (!cycleScore) {
                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(PI/2).thetaKp(3.5));
//                robot.setTargetPoint(138, 86, PI/2);

                addPacket("path", "going to deposit right rn");

                if (robot.y <= 91) {
                    robot.depositApproval = true;

                    if (time.seconds() > cycleScoreTime && !robot.intakeRev && !robot.depositingFreight/*time.seconds() > cycleScoreTime + 1.5*/) {
                        cycleCounter++;

                        if (30 - (System.currentTimeMillis() - robot.startTime) / 1000 > goToWarehouseTime + cycleScoreTime + parkTime + 1) {
                            goToWarehouseWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, PI / 2/*robot.theta*/, 10, 2, 0, 0),
                                    new Waypoint(140, 111, PI / 2, 10, -2, 0, goToWarehouseTime)
                            };
                            goToWarehousePath = new Path(goToWarehouseWaypoints);
                            goToWarehouse = false;
                        } else {
                            Waypoint[] parkWaypoints = new Waypoint[] {
                                    new Waypoint(robot.x, robot.y, PI / 2/*robot.theta*/, 30, 5, 0, 0),
                                    new Waypoint(140, 111, PI / 2, 20, -5, 0, parkTime)
                            };
                            parkPath = new Path(parkWaypoints);

                            cycleScore = true;
                        }

                        time.reset();
                    }
                }
            } else if (!park) {
                robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime, time.seconds())));

                addPacket("path", "parking right rn");

                if (time.seconds() > parkTime || robot.y > 100) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }
            } else {
                robot.drivetrain.stop();
            }

            robot.update();
        }
        robot.stop();
//        try {
//            detector.stop();
//        } catch (Exception ignore) {}
    }
}
