package org.firstinspires.ftc.teamcode.AutoPrograms.Red;

import static java.lang.Math.PI;
import static java.lang.Math.scalb;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
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

        // Segment Times
        double preloadScoreTime = 1.5;
        double goToWarehouseTime1 = 1.1;
        double goToWarehouseTime2 = 1.9;
        double cycleScoreTime1 = .6;
        double cycleScoreTime2 = 2;
        double parkTime1 = 1.2;
        double parkTime2 = 1.9;

        double depositTime = 2;
        double intakeTime = 2;
        boolean firstCycle = true;

        boolean generateGoToWarehousePath = true;
        boolean generateCycleScorePath = true;

        // Paths
        Path preloadScorePath = null;
        Path cycleScorePath = null;
        Path goToWarehousePath = null;
        Path parkPath = null;

        double[][] preloadScoreCoord = {{117, 60}, {120, 60}, {130, 60}};
        double[] preloadScoreCur = new double[1];

        waitForStart();

        // Put Vision Stuff Here
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right

        // Paths
        Waypoint[] preloadScoreWaypoints;

        if (barcodeCase == 0) {
            preloadScoreWaypoints = new Waypoint[]{
                    new Waypoint(135, 84, 0, -10, .1, 2, 0.0),
            };
            preloadScoreCur[0] = preloadScoreCoord[0][0];
            preloadScoreCur[1] = preloadScoreCoord[0][1];

        } else if (barcodeCase == 1) {

            preloadScoreWaypoints = new Waypoint[]{
                    new Waypoint(135, 84, 0, -10, .1, 10, 0.0),
                    new Waypoint(preloadScoreCoord[1][0], preloadScoreCoord[1][1], PI, 20, .1, 3, preloadScoreTime)
            };
            preloadScoreCur[0] = preloadScoreCoord[1][0];
            preloadScoreCur[1] = preloadScoreCoord[1][1];
        } else {
            preloadScoreWaypoints = new Waypoint[]{
                    new Waypoint(135, 84, 0, -10, .1, 10, 0.0),
                    new Waypoint(preloadScoreCoord[2][0], preloadScoreCoord[2][1], PI, 20, .1, 3, preloadScoreTime)
            };
            preloadScoreCur[0] = preloadScoreCoord[2][0];
            preloadScoreCur[1] = preloadScoreCoord[2][1];

        }
        preloadScorePath = new Path(new ArrayList<>(Arrays.asList(preloadScoreWaypoints)));



        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (opModeIsActive()){
                robot.setTargetPoint(preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds())));
                if (time.seconds() > preloadScoreTime){
                    break;
                }
            }
        time.reset();
        while (goToWarehouse && cycleScore && time.seconds()<cycleScoreTime2+goToWarehouseTime2+parkTime2){
            if(!goToWarehouse) {
                if (generateGoToWarehousePath) {
                    generateGoToWarehousePath = false;
                    Waypoint[] goToWarehouseWaypoints;
                    if (firstCycle) {
                        firstCycle = false;
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 40, 40, .1, 0),
                                new Waypoint(preloadScoreCur[0] + 20, preloadScoreCur[1] + 40, PI / 2, 40, .1, 0, goToWarehouseTime1),
                                new Waypoint(137, 120.19, PI / 2, 20, 0, 0, goToWarehouseTime2)
                        };
                    } else {
                        goToWarehouseWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, robot.theta, 40, 40, .1, 0),
                                new Waypoint(117 + 20, 110 + 40, PI / 2, 40, .1, 0, goToWarehouseTime1),
                                new Waypoint(137, 120.19, PI / 2, 20, 0, 0, goToWarehouseTime2)
                        };
                    }
                    goToWarehousePath = new Path(new ArrayList<>(Arrays.asList(goToWarehouseWaypoints)));

                }
                robot.setTargetPoint(goToWarehousePath.getRobotPose(Math.min(goToWarehouseTime2, time.seconds())));
                if (time.seconds() > goToWarehouseTime2) {
                    goToWarehouse = true;
                    time.reset();
                    break;
                }
            }
            else if (!cycleScore){
                if (generateCycleScorePath) {
                    Waypoint[] cycleScoreWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, robot.theta, -20, .1, .1, 0),
                            new Waypoint(137, 120.19 - 20, -PI / 2, 20, .1, .5, cycleScoreTime1),
                            new Waypoint(117, 60, -PI, 40, 20, .1, cycleScoreTime2)
                    };
                    cycleScorePath = new Path(new ArrayList<>(Arrays.asList(cycleScoreWaypoints)));
                }
                robot.setTargetPoint(cycleScorePath.getRobotPose(Math.min(cycleScoreTime2,time.seconds())));
                if(time.seconds()>cycleScoreTime2){
                    cycleScore = true;
                    time.reset();
                    break;
                }
            } else {
                goToWarehouse = false;
                cycleScore = false;

                generateGoToWarehousePath = true;
                generateCycleScorePath = true;
            }
        }

        Waypoint[] parkWaypoints = new Waypoint[]{
                new Waypoint(robot.x, robot.y, robot.theta, 40, 40, .1, 0),
                new Waypoint(117 + 20, 60 + 30, -PI / 2, -30, .1, .1, parkTime1),
                new Waypoint(135, 110, PI / 2, 10, 5, 0, parkTime2)
        };
        parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

        while(opModeIsActive()){
            robot.setTargetPoint(parkPath.getRobotPose(Math.min(parkTime2,time.seconds())));
            if (time.seconds()>parkTime2){
                break;
            }
        }
    }
}
