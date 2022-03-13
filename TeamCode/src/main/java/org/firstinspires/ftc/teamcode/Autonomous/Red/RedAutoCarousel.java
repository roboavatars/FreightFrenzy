package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodeDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Disabled
@Autonomous(name = "1 Red Auto Carousel", preselectTeleOp = "0 Red Teleop", group = "Red")
public class RedAutoCarousel extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Right;
    private boolean warehousePark = true;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            spin carousel
            intake + deliver duck on alliance shipping hub
            park in warehouse
        */

        Robot robot = new Robot(this, 135, 36, 0, true, true);
        robot.logger.startLogging(true, true);

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, true);
        barcodeDetector.start();

        // Segments
        boolean preloadScore = false;
        boolean spinCarousel = false;
        boolean deliverDuck = false;
        boolean park = false;

        // Segment Times
        double spinCarouselTime = 3;
        double parkTime = 3;

        // Paths
        Path parkPath = null;

        double carouselStartTime = 0;
        boolean carouselStart = false;

        waitForStart();

        barcodeCase = barcodeDetector.getResult();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.allianceLow;
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.allianceMid;
        } else {
            robot.cycleHub = Robot.DepositTarget.allianceHigh;
        }
        robot.deposit.preload = true;
        robot.carouselAuto = true;
        Robot.log("Barcode Case: " + barcodeCase);
        try {
            barcodeDetector.stop();
        } catch (Exception ignore) {}

        robot.intake.flipDown();
        robot.depositingFreight = true;
        robot.depositApproval = true;

        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            if (!preloadScore) {
                robot.drivetrain.setGlobalControls(0, 0, 0);

                addPacket("path", "initial deposit imo");

                if (robot.slidesInCommand) {
                    robot.cycleHub = Robot.DepositTarget.duck;
                    robot.deposit.preload = false;

                    time.reset();
                    preloadScore = true;
                }
            }

            // Go to Carousel
            else if (!spinCarousel) {
                robot.setTargetPoint(130, 24, -PI/4);

                if (robot.isAtPose(130, 24, -PI/4) && !carouselStart) {
                    robot.carousel.on();
                    carouselStart = true;
                    carouselStartTime = time.seconds();
                }

                if (carouselStart && time.seconds() - carouselStartTime > spinCarouselTime) {

                    robot.carousel.stop();
                    robot.intakeApproval = true;
                    robot.depositApproval = true;

                    spinCarousel = true;
                    time.reset();
                }
            }

            // Deliver Duck
            else if (!deliverDuck) {

                if (robot.slidesInCommand) {

                    if (warehousePark) {
                        Waypoint[] parkWaypoints = {
                                new Waypoint(robot.x, robot.y, robot.theta, 10, 30, 0, 0),
                                new Waypoint(140, 50, PI, 20, 30, 0, 1),
                                new Waypoint(140, 105, PI, 5, 5, 0, parkTime)
                        };
                        parkPath = new Path(parkWaypoints);
                    }

                    deliverDuck = true;
                    time.reset();
                }
            }

            // park
            else if (!park) {
                if (warehousePark) {
                    Pose curPose = parkPath.getRobotPose(Math.min(time.seconds(), parkTime));
                    robot.setTargetPoint(new Target(curPose).theta(curPose.theta+PI));
                } else {
                    robot.setTargetPoint(36, 18, 0);
                }

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
    }
}
