package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous(name = "0 Red Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoWarehouse extends LinearOpMode {

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

//        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;

        // Segment Times
        double preloadScoreTime = 1.75;
        double goToWarehouseTime1 = 1.5;
        double goToWarehouseTime2 = 2.5;
        double cycleScoreTime1 = 1;
        double cycleScoreTime2 = 3;
        double parkTime1 = 1;
        double parkTime2 = 2;

        double cycleX = 137;
        double depositX = 117;
        double depositY = 66;
        double depositTh = PI/10;

        //double[][] preloadScoreCoord = {{123, 70}, {120, 60}, {130, 60}};

        // Paths
        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(135, 78.5, 0, -10, -20, 0, 0),
                new Waypoint(depositX, depositY, depositTh+PI, 20, 5, 0, preloadScoreTime)
        };
        Path preloadScorePath = new Path(preloadScoreWaypoints);
        Path cycleScorePath = null;
        Path goToWarehousePath = null;
        Path parkPath = null;

        waitForStart();

//        int barcodeCase = detector.getAprilTagPipe().getResult();
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right
        Robot.log("Barcode Case: " + barcodeCase);
        if (barcodeCase == 0) {
            robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
        } else if (barcodeCase == 1) {
            robot.deposit.moveSlides(1, Deposit.deposit_height.MID);
        } else {
            robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
        }

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            if (!preloadScore) {
                Pose curPose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                if (robot.isAtPose(depositX, depositY) || time.seconds() > preloadScoreTime + 0.75) {
                    robot.deposit.open();
                }

                if (time.seconds() > preloadScoreTime + 1.5) {
                    Waypoint[] goToWarehouseWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, PI/4, 40, 40, 0, 0),
                            new Waypoint(cycleX, 84, PI/2, 20, 5, 0, goToWarehouseTime1),
                            new Waypoint(cycleX, 125, PI/2, 20, 0, 0, goToWarehouseTime2)
                    };
                    goToWarehousePath = new Path(goToWarehouseWaypoints);

                    time.reset();
                    preloadScore = true;
                }
            } else if (!goToWarehouse) {
                robot.setTargetPoint(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime2, time.seconds())));

                if (time.seconds() > 1) {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                    robot.deposit.close();
                }

                if (robot.y > 100) {
                    robot.intake.on();
                }

                if (time.seconds() > goToWarehouseTime2 + 1) {
                    Waypoint[] cycleScoreWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, robot.theta, -20, 5, 0, 0),
                            new Waypoint(cycleX, 84, -PI/2, 20, 5, 0, cycleScoreTime1),
                            new Waypoint(depositX, depositY, depositTh+PI, 40, 20, 0, cycleScoreTime2)
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                }
            } else if (!cycleScore) {
                Pose pose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime2, time.seconds()));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (robot.y < 92) {
                    robot.intake.off();
                    robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                    robot.deposit.hold();
                } else if (time.seconds() > 0.5) {
                    robot.intake.reverse();
                }

                if (robot.isAtPose(depositX, depositY) || time.seconds() > cycleScoreTime2 + 0.75) {
                    robot.deposit.open();
                }

                if (time.seconds() > cycleScoreTime2 + 1.5) {
                    if (30 - (System.currentTimeMillis() - robot.startTime) / 1000 > goToWarehouseTime2 + cycleScoreTime2 + parkTime2 + 1) {
                        Waypoint[] goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, PI/4, 40, 40, 0, 0),
                                new Waypoint(cycleX, 84, PI/2, 20, 5, 0, goToWarehouseTime1),
                                new Waypoint(cycleX, 125, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                        goToWarehousePath = new Path(goToWarehouseWaypoints);
                        goToWarehouse = false;
                    } else {
                        Waypoint[] parkWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
                                new Waypoint(cycleX, 84, PI/2, 20, 5, 0, parkTime1),
                                new Waypoint(cycleX, 110, -PI/2, -10, 0, 0, parkTime2)
                        };
                        parkPath = new Path(parkWaypoints);

                        cycleScore = true;
                    }

                    time.reset();
                }
            } else if (!park) {
                robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                robot.deposit.close();
                robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime2, time.seconds())));
                if (time.seconds() > parkTime2) {
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
