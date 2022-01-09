package org.firstinspires.ftc.teamcode.Autonomous.Red;

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
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous(name = "0 Red Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoWarehouse extends LinearOpMode {
    public static int barcodeCase = 2; // 0 = left, 1 = mid, 2 = right


    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 135, 78.5, 0, true, true);
        robot.logger.startLogging(true, true);

        BarcodeDetector detector = new BarcodeDetector(this, true);
        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;

        // Segment Times
        double preloadScoreTime = 1.75;
        double goToWarehouseTime1 = 2;
        double goToWarehouseTime2 = 5;
        double cycleScoreTime1 = 2;
        double cycleScoreTime2 = 3.5;
        double parkTime1 = 2;
        double parkTime2 = 3.5;

        double cycleX = 138;
        double depositX = 117;
        double depositY = 68;
        double depositTh = PI/10;

        double[][] preloadScoreCoord = {{111.5, 63, 5*PI/6}, {115.5, 63, 5*PI/6}, {119, 63, 5*PI/6}};

        // Paths
        Path preloadScorePath = null;
        Path cycleScorePath = null;
        Path goToWarehousePath = null;
        Path parkPath = null;

        int cycleCounter = 0;

        waitForStart();

        if(detector.getResult() == BarcodePipeline.Case.Left){
            barcodeCase = 0;
        } else if(detector.getResult() == BarcodePipeline.Case.Middle){
            barcodeCase = 1;
        } else {
            barcodeCase = 2;
        }
        Robot.log("Barcode Case: " + barcodeCase);

        if (barcodeCase == 0) {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.LOW);
        } else if (barcodeCase == 1) {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.MID);
        } else {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.TOP);
        }
        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(135, 78.5, 0, -10, -20, 0, 0),
                new Waypoint(preloadScoreCoord[barcodeCase][0], preloadScoreCoord[barcodeCase][1], preloadScoreCoord[barcodeCase][2], 20, 5, 0, preloadScoreTime)
        };
        preloadScorePath = new Path(preloadScoreWaypoints);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            if (!preloadScore) {
                Pose curPose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                if (robot.isAtPose(depositX, depositY) || time.seconds() > preloadScoreTime + 0.5) {
//                    robot.deposit.autoOpen();
                }

                if (time.seconds() > preloadScoreTime + 1.5) {
                    Waypoint[] goToWarehouseWaypoints;
                    if (barcodeCase == 0) {
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, preloadScoreCoord[barcodeCase][2], 40, 40, 0, 0),
                                new Waypoint(130, 70, 5*PI/12, 20, 5, -1, goToWarehouseTime1-1.5),
                                new Waypoint(cycleX, 84, PI/2, 20, 5, -1, goToWarehouseTime1),
                                new Waypoint(cycleX + 10, 122, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                    } else if (barcodeCase == 1) {
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, preloadScoreCoord[barcodeCase][2], 40, 40, 0, 0),
                                new Waypoint(130, 70, 5*PI/12, 20, 5, -1, goToWarehouseTime1-1.5),
                                new Waypoint(cycleX, 84, PI/2, 20, 5, -1, goToWarehouseTime1),
                                new Waypoint(cycleX + 10, 122, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                    } else {
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, preloadScoreCoord[barcodeCase][2], 40, 40, 0, 0),
                                new Waypoint(130, 70, 5*PI/12, 20, 5, -1, goToWarehouseTime1-1.5),
                                new Waypoint(cycleX, 84, PI/2, 20, 5, -1, goToWarehouseTime1),
                                new Waypoint(cycleX + 10, 122, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                    }
                    goToWarehousePath = new Path(goToWarehouseWaypoints);

                    time.reset();
                    preloadScore = true;
                }
            } else if (!goToWarehouse) {
                robot.setTargetPoint(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime2, time.seconds())));

                if (time.seconds() > goToWarehouseTime1 && time.seconds() < goToWarehouseTime1 + 0.1) {
                    robot.resetOdo(cycleX, robot.y, PI/2);
                }
                if (time.seconds() > goToWarehouseTime2) {
                    robot.resetOdo(cycleX, robot.y, 5*PI/12);
                }

                if (time.seconds() > 1) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
//                    robot.deposit.close();
                }

                if (robot.y > 100) {
//                    robot.intake.on();
                }

                if (time.seconds() > goToWarehouseTime2 + .5) {
                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(robot.x, robot.y, robot.theta, -20, 5, 0, 0),
                            new Waypoint(cycleX + 2, 86, -PI/2, 20, 5, 0, cycleScoreTime1),
                            new Waypoint(depositX, depositY, depositTh + PI, 40, 20, -2 * cycleCounter, cycleScoreTime2)
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                }
            } else if (!cycleScore) {
                Pose pose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime2, time.seconds()));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (time.seconds() > cycleScoreTime1 && time.seconds() < cycleScoreTime1 + 0.1) {
                    robot.resetOdo(cycleX, robot.y, PI/2);
                }

                if (robot.y < 92) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.TOP);
//                    robot.deposit.hold();
                }

                if (time.seconds() > 1.5) {
//                    robot.intake.off();
                } else if (time.seconds() > 0.75) {
//                    robot.intake.reverse();
                }

                if (robot.isAtPose(depositX, depositY, depositTh) || time.seconds() > cycleScoreTime2 + 0.75) {
//                    robot.deposit.open();
                }

                if (time.seconds() > cycleScoreTime2 + 1.5) {
                    cycleCounter++;
                    if (30 - (System.currentTimeMillis() - robot.startTime) / 1000 > goToWarehouseTime2 + cycleScoreTime2 + parkTime2 + 1) {
                        Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, depositTh, 40, 40, 0, 0),
                                new Waypoint(cycleX+2, 84, PI/2, 20, 5, -1, goToWarehouseTime1/* + .5* cycleCounter*/),
                                new Waypoint(cycleX + 12, 122 + 3 * cycleCounter, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                        goToWarehousePath = new Path(goToWarehouseWaypoints);
                        goToWarehouse = false;
                    } else {
                        Waypoint[] parkWaypoints = new Waypoint[] {
                                new Waypoint(robot.x, robot.y, depositTh, 40, 40, 0, 0),
                                new Waypoint(cycleX + 7.5, 84, 5*PI/12, 20, 5, -1, parkTime1),
                        };
                        parkPath = new Path(parkWaypoints);

                        cycleScore = true;
                    }

                    time.reset();
                }
            } else if (!park) {
                if (time.seconds() > 1) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
//                    robot.deposit.close();
                }

                if (time.seconds() > parkTime1){
                    robot.drivetrain.setRawPower(0.5, 1, 0.5, 1);
                }
                robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime2, time.seconds())));
                if (time.seconds() > parkTime2 || robot.y > 100) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }
            } else {
                robot.drivetrain.stop();
            }

            robot.update();
        }

        robot.stop();
        try {
            detector.stop();
        } catch (Exception ignore) {}
    }
}