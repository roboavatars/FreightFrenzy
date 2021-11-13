package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static java.lang.Math.PI;
import static java.lang.Math.scalb;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

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

        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        int cycleCounter = 0;
        boolean cycle = false;

        // Segment Times
        double preloadScoreTime = 1.5;
        double goToWarehouseTime1 = 1.1;
        double goToWarehouseTime2 = 1.9;
        double cycleScoreTime1 = .6;
        double cycleScoreTime2 = 2;
        double parkTime1 = 1.2;
        double parkTime2 = 1.9;

        int numOfCycles = 5; //<<<<<<<<<<<<<

        double depositTime = 2;
        double intakeTime = 2;

        // Paths
        Path preloadScorePath = null;
        Path cycleScorePath = null;
        Path goToWarehousePath = null;
        Path parkPath = null;

        double[][] preloadScoreCoord = {{117, 60}, {120, 60}, {130, 60}};


        waitForStart();

        // Put Vision Stuff Here
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right

        // waypoint arrays
        Waypoint[] preloadScoreWaypoints;
        Waypoint[] goToWarehouseWaypoints;
        Waypoint[] cycleScoreWaypoints;


        preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(135, 84, 0, -10, -20, 2, 0.0),
                new Waypoint(preloadScoreCoord[barcodeCase][0], preloadScoreCoord[barcodeCase][1], PI, 20, 5, 3, preloadScoreTime)
        };
        preloadScorePath = new Path(new ArrayList<>(Arrays.asList(preloadScoreWaypoints)));


        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opModeIsActive()) {
            if (!preloadScore) {
                Pose pose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(pose).theta(pose.theta+PI));
                if (time.seconds() > preloadScoreTime) {
                    goToWarehouseWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, 0, 40, 40, .1, 0),
                            new Waypoint(117 + 20, 60 + 30, PI / 2, 40, .1, 0, goToWarehouseTime1),
                            new Waypoint(137, 120.19, PI / 2, 20, 0, 0, goToWarehouseTime2)
                    };
                    goToWarehousePath = new Path(new ArrayList<>(Arrays.asList(goToWarehouseWaypoints)));

                    time.reset();
                    preloadScore = true;
                }
            } else if (!cycle) {
                if (!goToWarehouse) {
                    robot.setTargetPoint(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime2, time.seconds())));
                    if (time.seconds() > goToWarehouseTime2) {
                        cycleScoreWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, -PI/2, -20, .1, .1, 0),
                                new Waypoint(137, 120.19 - 20, -PI / 2, 20, .1, .5, cycleScoreTime1),
                                new Waypoint(117, 60, PI, 40, 20, .1, cycleScoreTime2)
                        };
                        cycleScorePath = new Path(new ArrayList<>(Arrays.asList(cycleScoreWaypoints)));

                        time.reset();
                        goToWarehouse = true;
                    }
                } else if (!cycleScore) {
                    Pose pose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime2, time.seconds()));
                    robot.setTargetPoint(new Target(pose).theta(pose.theta+PI));
                    if (time.seconds() > cycleScoreTime2) {
                        time.reset();
                        cycleScore = true;
                    }

                } else {
                    cycleCounter += 1;
                    if (cycleCounter >= numOfCycles) {
                        Waypoint[] parkWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, 0, 40, 40, .1, 0),
                                new Waypoint(117 + 20, 60 + 40, PI / 2, 20, .1, .1, parkTime1),
                                new Waypoint(135, 110, -PI / 2, -10, 5, 0, parkTime2)
                        };
                        parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

                        time.reset();
                        cycle = true;
                    } else {
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, 0, 40, 40, .1, 0),
                                new Waypoint(117 + 20, 60 + 30, PI / 2, 40, .1, 0, goToWarehouseTime1),
                                new Waypoint(137, 120.19, PI / 2, 20, 0, 0, goToWarehouseTime2)
                        };
                        goToWarehousePath = new Path(new ArrayList<>(Arrays.asList(goToWarehouseWaypoints)));

                        goToWarehouse = false;
                        cycleScore = false;
                        time.reset();
                    }
                }
            } else {
                robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime2, time.seconds())));
                if (time.seconds()>parkTime2){
                    break;
                }
            }
        }
    }
}