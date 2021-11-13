package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

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

//        Vision detector = new Vision(this, Vision.Pipeline.AprilTag);
//        detector.start();

        // Segments
        boolean deliverPreloadedFreight = false;
        boolean spinCarousel = false;
        boolean deliverDuck = false;
        boolean goToWarehouse = false;
        boolean cycle = false;
        boolean park = false;

        // Segment Times
        double deliverPreloadedFreightTime = 2.0;
        double spinCarouselTime = 1.5;
        double deliverDuckTime = 1.5;
        double goToWarehouseTime = 1.5;
        double cycleTime = 4.0;
        double parkTime = 1.5;

        // Paths
        Path deliverPreloadedFreightPath = null;
        Path spinCarouselPath = null;
        Path deliverDuckPath = null;
        Path goToWarehousePath = null;
        Path cyclePath = null;
        Path parkPath = null;

        waitForStart();

//        int barcodeCase = detector.getAprilTagPipe().getResult();
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right
        Robot.log("Barcode Case: " + barcodeCase);

        // Paths
        if (barcodeCase == 0) {
            deliverPreloadedFreightTime = 2.0;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(115, 51, 7*PI/4, 10, -10, 0, deliverPreloadedFreightTime),
            };
            deliverPreloadedFreightPath = new Path(deliverPreloadedFreightWaypoints);

        } else if (barcodeCase == 1) {
            deliverPreloadedFreightTime = 2.0;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(116, 54, 0, 10, -10, 0, deliverPreloadedFreightTime),
            };
            deliverPreloadedFreightPath = new Path(deliverPreloadedFreightWaypoints);
        } else {
            deliverPreloadedFreightTime = 2.25;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(120, 57, 0, 10, -10, 0, deliverPreloadedFreightTime),
            };
            deliverPreloadedFreightPath = new Path(deliverPreloadedFreightWaypoints);
        }

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            // Deliver Preloaded Freight
            if(!deliverPreloadedFreight) {
                robot.setTargetPoint(new Target(deliverPreloadedFreightPath.getRobotPose(Math.min(time.seconds(), deliverPreloadedFreightTime))));

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
                        new Waypoint(130, 15, 7*PI/4, -20, -10, 0, spinCarouselTime),
                };
                spinCarouselPath = new Path(spinCarouselWaypoints);

                deliverPreloadedFreight = true;
                time.reset();
            }

            // Go to Carousel
            else if (!spinCarousel) {
                robot.setTargetPoint(new Target(spinCarouselPath.getRobotPose(Math.min(time.seconds(), spinCarouselTime))));

                // spin
                robot.carousel.rotate();
                time.reset();

                if (time.milliseconds() > 1000) {
                    robot.carousel.stop();
                }

                // Intake rings
                robot.intake.on();

                Waypoint[] deliverDuckWaypoints = new Waypoint[] {
                        new Waypoint(130, 15, 3*PI/4, -20, -10, 0, 0),
                        new Waypoint(120, 60, 0, -20, -10, 0, deliverDuckTime),
                };
                deliverDuckPath = new Path(deliverDuckWaypoints);

                spinCarousel = true;
                time.reset();
            }

            // Deliver Duck
            else if (!deliverDuck) {
                robot.setTargetPoint(new Target(deliverDuckPath.getRobotPose(Math.min(time.seconds(), deliverDuckTime))));

                robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                robot.deposit.open();

                time.reset();

                if (time.milliseconds() > 300) {
                    robot.deposit.moveSlides(1, Deposit.deposit_height.HOME);
                    robot.deposit.close();
                }

                Waypoint[] goToWarehouseWaypoints = new Waypoint[] {
                        new Waypoint(120, 60, 0, -20, -10, 0, 0),
                        new Waypoint(135, 78, PI/2, -20, -10, 0, 1),
                        new Waypoint(135, 110, PI/2,-20,-10,0,goToWarehouseTime),
                };
                goToWarehousePath = new Path(goToWarehouseWaypoints);
                deliverDuck = true;
                time.reset();
            }

            else if (!goToWarehouse) {
                robot.setTargetPoint(new Target(goToWarehousePath.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));

                robot.intake.on();

                Waypoint[] cycleWaypoints = new Waypoint[] {
                        new Waypoint(135, 110, PI/2,-20,-10,0,0),
                        new Waypoint(135, 78, PI/2,-20,-10,0,0.5),
                        new Waypoint(116, 73, PI/5,-20,-10,0,1.5),

                };
                cyclePath = new Path(cycleWaypoints);

                goToWarehouse = true;
                time.reset();
            }

            // cycle freight
            else if (!cycle) {
                if ((System.currentTimeMillis() - robot.startTime) > 3000) {
                    Pose pose = cyclePath.getRobotPose(Math.min(time.seconds(), cycleTime));
                    robot.setTargetPoint(new Target(pose).theta(pose.theta + PI/2));

                    robot.deposit.moveSlides(1, Deposit.deposit_height.TOP);
                    robot.deposit.open();
                    time.reset();

                    if (time.milliseconds() > 300) {
                        robot.deposit.close();
                    }

                    robot.setTargetPoint(new Target(goToWarehousePath.getRobotPose(Math.min(time.seconds(), goToWarehouseTime))));
                } else {
                    cycle = true;
                    time.reset();
                }
            }

            // park
            else if (!park) {
                double curTime = Math.min(time.seconds(), parkTime);
                Pose curPose = parkPath.getRobotPose(curTime);

                robot.setTargetPoint(114,111,PI/2);

                if (time.seconds() > parkTime) {
                    Robot.log("Auto finished in " + ((System.currentTimeMillis() - robot.startTime) / 1000) + " seconds");

                    park = true;
                }
            }

            else {
//                robot.drivetrain.stop();
                break;
            }

            sendPacket();

//            robot.update();
        }

        robot.stop();
//        try {
//            detector.stop();
//        } catch (Exception ignore) {}
    }
}
