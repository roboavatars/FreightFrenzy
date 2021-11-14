package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@Autonomous(name = "Red Auto Warehouse", preselectTeleOp = "1 Teleop", group = "Red")
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

        Robot robot = new Robot(this, 135, 84, 0, true, true);
        robot.logger.startLogging(true, true);

//        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;

        // Segment Times
        double preloadScoreTime = 1.5;
        double goToWarehouseTime1 = 1.1;
        double goToWarehouseTime2 = 1.9;
        double cycleScoreTime1 = 0.6;
        double cycleScoreTime2 = 2.0;
        double parkTime1 = 1.2;
        double parkTime2 = 1.9;

        // Paths
        Path preloadScorePath = null;
        Path cycleScorePath = null;
        Path goToWarehousePath = null;
        Path parkPath = null;

        double[][] preloadScoreCoord = {{117, 60}, {120, 60}, {130, 60}};

        waitForStart();

//        int barcodeCase = detector.getAprilTagPipe().getResult();
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right
        Robot.log("Barcode Case: " + barcodeCase);

        Waypoint[] preloadScoreWaypoints = new Waypoint[] {
                new Waypoint(135, 84, 0, -10, -20, 2, 0.0),
                new Waypoint(preloadScoreCoord[barcodeCase][0], preloadScoreCoord[barcodeCase][1], PI, 20, 5, 0, preloadScoreTime)
        };
        preloadScorePath = new Path(preloadScoreWaypoints);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            if (!preloadScore) {
                Pose pose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));
                if (time.seconds() > preloadScoreTime) {
                    Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                            new Waypoint(/*preloadScoreCoord[barcodeCase][0], preloadScoreCoord[barcodeCase][1]*/robot.x, robot.y, 0, 40, 40, 0, 0),
                            new Waypoint(137, 90, PI/2, 40, .1, 0, goToWarehouseTime1),
                            new Waypoint(137, 120, PI/2, 20, 0, 0, goToWarehouseTime2)
                    };
                    goToWarehousePath = new Path(goToWarehouseWaypoints);

                    time.reset();
                    preloadScore = true;
                }
            } else if (!goToWarehouse) {
                robot.setTargetPoint(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime2, time.seconds())));
                if (time.seconds() > goToWarehouseTime2) {
                    Waypoint[] cycleScoreWaypoints = new Waypoint[] {
                            new Waypoint(/*137, 120*/robot.x, robot.y, PI/2, -20, .1, 0, 0),
                            new Waypoint(137, 100, -PI/2, 20, .1, .5, cycleScoreTime1),
                            new Waypoint(117, 60, PI, 40, 20, 0, cycleScoreTime2)
                    };
                    cycleScorePath = new Path(cycleScoreWaypoints);

                    time.reset();
                    goToWarehouse = true;
                }
            } else if (!cycleScore) {
                Pose pose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime2, time.seconds()));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (time.seconds() > cycleScoreTime2) {
                    if (30 - (System.currentTimeMillis() - robot.startTime) / 1000 > goToWarehouseTime2 + cycleScoreTime2 + parkTime2 + 1) {
                        Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                                new Waypoint(/*117, 60*/robot.x, robot.y, 0, 40, 40, 0, 0),
                                new Waypoint(137, 90, PI/2, 40, .1, 0, goToWarehouseTime1),
                                new Waypoint(137, 120, PI/2, 20, 0, 0, goToWarehouseTime2)
                        };
                        goToWarehousePath = new Path(goToWarehouseWaypoints);

                        goToWarehouse = false;
                    } else {
                        Waypoint[] parkWaypoints = new Waypoint[] {
                                new Waypoint(/*117, 60*/robot.x, robot.y, 0, 40, 40, 0, 0),
                                new Waypoint(137, 100, PI/2, 20, .1, 0, parkTime1),
                                new Waypoint(135, 110, -PI/2, -10, 5, 0, parkTime2)
                        };
                        parkPath = new Path(parkWaypoints);

                        cycleScore = true;
                    }

                    time.reset();
                }
            } else if (!park) {
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
