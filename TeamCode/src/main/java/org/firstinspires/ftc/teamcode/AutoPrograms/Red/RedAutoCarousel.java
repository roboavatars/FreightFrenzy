package org.firstinspires.ftc.teamcode.AutoPrograms.Red;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.PI;

@Config
@Autonomous(name = "Red Auto Carousel", preselectTeleOp = "1 Teleop", group = "Red")
public class RedAutoCarousel extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            carousel
            intake duck
            deliver duck on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 135, 36, PI, true, true);
        robot.logger.startLogging(true, true);

        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
        detector.start();

        // Segments
        boolean deliverPreloadedFreight = false;
        boolean spinCarousel = false;
        boolean deliverDuck = false;
        boolean goToWarehouse = false;
        boolean goToWarehouse2 = false;
        boolean cycleFreight = false;
        boolean cycle = false;
        boolean park = false;

        // Segment Times
        double detectBarcodeTime = 0.75;
        double deliverPreloadedFreightTime = 1.0;
        double spinCarouselTime = 1.5;
        double deliverDuckTime = 1.5;
        double goToWarehouseTime = 1.5;
        double goToWarehouseTime2 = 1.5;
        double cycleTime = 1.5;
        double parkTime = 1.5;

        int cycleCount = 0;

        // Paths
        Path deliverPreloadedFreightPath = null;
        Path spinCarouselPath = null;
        Path deliverDuckPath = null;
        Path goToWarehousePath = null;
        Path goToWarehousePath2 = null;
        Path cyclePath = null;
        Path parkPath = null;

        waitForStart();

        // Put Vision Stuff Here
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right

        // Paths

        Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                new Waypoint(135, 36, PI, 20, 20, 0, 0),
                new Waypoint(135, 38, PI, 20, 20, 0, 0.5),
                new Waypoint(120, 58, PI, 20, 20, 0, deliverPreloadedFreightTime),
        };
        deliverPreloadedFreightPath = new Path(new ArrayList<>(Arrays.asList(deliverPreloadedFreightWaypoints)));

        detector.setPipeline(Vision.Pipeline.AprilTag);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            // Deliver Preloaded Freight
            if(!deliverPreloadedFreight) {
                Pose pose = deliverPreloadedFreightPath.getRobotPose(Math.min(time.seconds(), deliverPreloadedFreightTime));
                robot.setTargetPoint(new Target(pose).theta(pose.theta+PI));

                if (barcodeCase == 0) {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                    robot.deposit.open();
                } else if (barcodeCase == 1) {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.MID);
                    robot.deposit.open();
                } else {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                    robot.deposit.open();
                }

                robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                robot.deposit.close();

                Waypoint[] spinCarouselWaypoints = new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta, 40, 30, 0, 0),
                        new Waypoint(130, 15, 7*PI/4, 30, -10, 0, spinCarouselTime),
                };
                spinCarouselPath = new Path(new ArrayList<>(Arrays.asList(spinCarouselWaypoints)));

                deliverPreloadedFreight = true;
                time.reset();
            }

            // Go to Carousel
            else if (!spinCarousel) {
                robot.setTargetPoint(new Target(spinCarouselPath.getRobotPose(Math.min(time.seconds(), spinCarouselTime))));

                // spin
                robot.carousel.rotate();
                time.reset();

                if (time.seconds() > 1) {
                    robot.carousel.stop();
                }

                // Intake rings
                robot.intake.on();

                Waypoint[] deliverDuckWaypoints = new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta+PI, 20, 10, 0, 0),
                        new Waypoint(127, 20, 3*PI/5, 20, 10, 0, 0.5),
                        new Waypoint(124, 57, PI, 20, 5, 0, deliverDuckTime),
                };
                deliverDuckPath = new Path(new ArrayList<>(Arrays.asList(deliverDuckWaypoints)));

                spinCarousel = true;
                time.reset();
            }

            // Deliver Duck
            else if (!deliverDuck) {
                Pose pose = deliverDuckPath.getRobotPose(Math.min(time.seconds(), deliverDuckTime));
                robot.setTargetPoint(new Target(pose).theta(pose.theta+PI));

                robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                robot.deposit.open();

                time.reset();

                if (time.seconds() > 0.1) {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                    robot.deposit.close();
                }

                Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta, 35, 30, 0, 0),
                        new Waypoint(135, 78, PI/2, 20, 20, 0, 1),
                        new Waypoint(135, 115, PI/2,10,-5,0,goToWarehouseTime),
                };
                goToWarehousePath = new Path(new ArrayList<>(Arrays.asList(goToWarehouseWaypoints)));
                deliverDuck = true;
                time.reset();
            }

            else if (!goToWarehouse) {
                robot.setTargetPoint(new Target(goToWarehousePath.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));

                robot.intake.on();

                Waypoint[] cycleWaypoints = new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta,-10,-20,0,0),
                        new Waypoint(135, 82, PI/2,-20,-5,0,1),
                        new Waypoint(118, 63, PI/12,-20,-10,0,cycleTime),

                };
                cyclePath = new Path(new ArrayList<>(Arrays.asList(cycleWaypoints)));

                Waypoint[] goToWarehouseWaypoints2 = new Waypoint[] {
                        new Waypoint(118, 63, PI/12,10,10,0,0),
                        new Waypoint(135, 82, PI/2,10,10,0,1),
                        new Waypoint(135, 115, PI/2,10,10,0,cycleTime),

                };
                goToWarehousePath2 = new Path(new ArrayList<>(Arrays.asList(goToWarehouseWaypoints2)));

                goToWarehouse = true;
                time.reset();
            }

            // cycle freight
            else if (!cycleFreight) {
                if (!cycle) {
                    Pose pose = cyclePath.getRobotPose(Math.min(time.seconds(), cycleTime));
                    robot.setTargetPoint(new Target(pose).theta(pose.theta+PI));

                    robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                    robot.deposit.open();
                    time.reset();

                    if (time.seconds() > 0.1) {
                        robot.deposit.close();
                    }

                    robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                    cycleCount++;

                    cycle = true;
                    time.reset();
                }

                else if (!goToWarehouse2) {
                    robot.setTargetPoint(new Target(goToWarehousePath2.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));
                    robot.intake.on();

                    if (time.seconds() > 0.3) {
                        robot.intake.off();
                    }

                    goToWarehouse2 = true;
                    time.reset();
                }

                else {
                    if (cycleCount >= 5) {
                        cycle = true;
                        goToWarehouse2 = true;
                        cycleFreight = true;
                    } else {
                        cycle = false;
                        goToWarehouse2 = false;
                    }
                    time.reset();
                }

                Waypoint[] parkWaypoints = new Waypoint[] {
                        new Waypoint(robot.x, robot.y, robot.theta,10,10,0,0),
                        new Waypoint(114, 111, PI/2,10,10,0,parkTime),

                };
                parkPath = new Path(new ArrayList<>(Arrays.asList(parkWaypoints)));

            }

            // park
            else if (!park) {
                robot.setTargetPoint(new Target(parkPath.getRobotPose(Math.min(time.seconds(), parkTime))));

                if (time.seconds() > parkTime) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }

                if ((System.currentTimeMillis() - robot.startTime) >= 33000) {
                    Robot.log("Breaking Loop");
                    break;
                }

            }

            else {
                robot.drivetrain.stop();
                if (robot.notMoving() || (System.currentTimeMillis() - robot.startTime) >= 30000) {
                    break;
                }
            }

            robot.update();
        }

        robot.stop();
        try {
            detector.stop();
        } catch (Exception ignore) {}
    }
}
