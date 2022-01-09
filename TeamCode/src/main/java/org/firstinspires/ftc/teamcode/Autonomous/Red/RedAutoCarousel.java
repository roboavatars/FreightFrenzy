package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Disabled
@Autonomous(name = "1 Red Auto Carousel", preselectTeleOp = "1 Teleop", group = "Red")
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

        Robot robot = new Robot(this, 135, 36, 0, true, true);
        robot.logger.startLogging(true, true);

//        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
//        detector.start();

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

//        int barcodeCase = detector.getAprilTagPipe().getResult();
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right
        Robot.log("Barcode Case: " + barcodeCase);
        if (barcodeCase == 0) {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.TOP);
        } else if (barcodeCase == 1) {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.MID);
        } else {
//            robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
        }

        // Paths

        Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[]{
                new Waypoint(135, 36, 0, -10, -10, 0, 0),
                new Waypoint(128, 43, -PI / 3, -10, -10, 0, 0.5),
                new Waypoint(120, 58, 0, -10, 10, 0, deliverPreloadedFreightTime),
        };
        deliverPreloadedFreightPath = new Path(deliverPreloadedFreightWaypoints);

//        detector.setPipeline(Vision.Pipeline.AprilTag);

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            // Deliver Preloaded Freight
            if (!deliverPreloadedFreight) {
                addPacket("path", "deliverPreloadedFreight");
                Pose pose = deliverPreloadedFreightPath.getRobotPose(Math.min(time.seconds(), deliverPreloadedFreightTime));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (time.seconds() > deliverPreloadedFreightTime) {

                    Waypoint[] spinCarouselWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, 0, 40, 30, 0, 0),
                            new Waypoint(130, 15, 7 * PI / 4, 30, -10, 0, spinCarouselTime),
                    };
                    spinCarouselPath = new Path(spinCarouselWaypoints);

                    deliverPreloadedFreight = true;
                    time.reset();
                }
            }

            // Go to Carousel
            else if (!spinCarousel) {
                robot.setTargetPoint(new Target(spinCarouselPath.getRobotPose(Math.min(time.seconds(), spinCarouselTime))));

                if (time.seconds() > 1) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
//                    robot.deposit.close();
                }

                if (time.seconds() > spinCarouselTime) {

                    // spin
                    robot.carousel.rotateRed();
                    time.reset();

                    if (time.seconds() > 1) {
                        robot.carousel.stop();
                    }
//                    robot.intake.on();

                    Waypoint[] deliverDuckWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, 3 * PI / 4, 20, 10, 0, 0),
                            new Waypoint(128, 35, 7 * PI / 5, -10, -10, 0, 0.5),
                            new Waypoint(125, 57, 0, -20, 5, 0, deliverDuckTime),
                    };
                    deliverDuckPath = new Path(deliverDuckWaypoints);

                    spinCarousel = true;
                    time.reset();
                }
            }

            // Deliver Duck
            else if (!deliverDuck) {
                Pose pose = deliverDuckPath.getRobotPose(Math.min(time.seconds(), deliverDuckTime));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (time.seconds() > deliverDuckTime) {

//                robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
//                robot.deposit.open();

                    Waypoint[] goToWarehouseWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, 0, 35, 30, 0, 0),
                            new Waypoint(138, 78, PI / 2, 20, 20, 0, 1),
                            new Waypoint(138, 115, PI / 2, 10, -5, 0, goToWarehouseTime),
                    };
                    goToWarehousePath = new Path(goToWarehouseWaypoints);
                    deliverDuck = true;
                    time.reset();
                }
            } else if (!goToWarehouse) {
                robot.setTargetPoint(new Target(goToWarehousePath.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));

                if (time.seconds() > 1) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
//                    robot.deposit.close();
                }

                if (robot.y > 100) {
//                    robot.intake.on();
                }

                if (time.seconds() > goToWarehouseTime) {

                    Waypoint[] cycleWaypoints = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, PI / 2, -10, -20, 0, 0),
                            new Waypoint(138, 82, PI / 2, -20, -5, 0, 1),
                            new Waypoint(118, 63, PI / 12, -20, -10, 0, cycleTime),

                    };
                    cyclePath = new Path(cycleWaypoints);

                    goToWarehouse = true;
                    time.reset();
                }
            }

            // cycle freight
            else if (!cycle) {
                Pose pose = cyclePath.getRobotPose(Math.min(time.seconds(), cycleTime));
                robot.setTargetPoint(new Target(pose).theta(pose.theta + PI));

                if (robot.y < 92) {
//                    robot.intake.off();
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.TOP);
//                    robot.deposit.hold();
                } else if (time.seconds() > 0.5) {
//                    robot.intake.reverse();
                }

                if (time.seconds() > cycleTime) {
                    Waypoint[] goToWarehouseWaypoints2 = new Waypoint[]{
                            new Waypoint(robot.x, robot.y, PI / 12, 10, 10, 0, 0),
                            new Waypoint(138, 82, PI / 2, 10, 10, 0, 1),
                            new Waypoint(138, 115, PI / 2, 10, 10, 0, cycleTime),

                    };
                    goToWarehousePath2 = new Path(goToWarehouseWaypoints2);

                    cycle = true;
                    time.reset();
                }
            } else if (!goToWarehouse2) {
                robot.setTargetPoint(new Target(goToWarehousePath2.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));

                if (time.seconds() > 1) {
//                    robot.deposit.moveSlides(1, Deposit.DepositHeight.HOME);
//                    robot.deposit.close();
                }

                if (robot.y > 100) {
//                    robot.intake.on();
                }

                if (time.seconds() > goToWarehouseTime2) {
                    if (30 - (System.currentTimeMillis() - robot.startTime) / 1000 > goToWarehouseTime2 + cycleTime + parkTime + 1) {
                        Waypoint[] cycleWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, PI / 2, -10, -20, 0, 0),
                                new Waypoint(138, 82, PI / 2, -20, -5, 0, 1),
                                new Waypoint(118, 63, PI / 12, -20, -10, 0, cycleTime),
                        };
                        cyclePath = new Path(cycleWaypoints);

                        cycle = false;
                        goToWarehouse2 = false;
                    } else {
                        Waypoint[] parkWaypoints = new Waypoint[]{
                                new Waypoint(robot.x, robot.y, PI / 2, 10, 10, 0, 0),
                                new Waypoint(114, 111, PI / 2, 10, 10, 0, parkTime),

                        };
                        parkPath = new Path(parkWaypoints);

                        goToWarehouse2 = true;
                    }

                    time.reset();
                }
            }

            // park
            else if (!park) {
                robot.setTargetPoint(new Target(parkPath.getRobotPose(Math.min(time.seconds(), parkTime))));

                if (time.seconds() > parkTime) {

                    if (time.seconds() > parkTime) {
                        Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                        park = true;
                    }
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
