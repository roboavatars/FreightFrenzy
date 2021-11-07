package org.firstinspires.ftc.teamcode.AutoPrograms.Red;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Vision;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;
import java.util.Arrays;

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
        boolean intakeDuck = false;
        boolean deliverDuck = false;
        boolean cycle = false;
        boolean park = false;

        // Segment Times
        double detectBarcodeTime = 0.75;
        double deliverPreloadedFreightTime = 2.0;
        double spinCarouselTime = 1.5;
        double intakeDuckTime = 1.0;
        double deliverDuckTime = 1.5;
        double cycleTime = 4.0;
        double parkTime = 1.5;

        // Paths
        Path deliverPreloadedFreightPath = null;
        Path spinCarouselPath = null;
        Path intakeDuckPath = null;
        Path deliverDuckPath = null;
        Path cyclePath = null;
        Path parkPath = null;

        waitForStart();

        // Put Vision Stuff Here
        int barcodeCase = 0; // 0 = left, 1 = mid, 2 = right

        // Paths

        if (barcodeCase == 0) {
            deliverPreloadedFreightTime = 2.0;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(104, 51, 3*PI/4, 5, 30, 0, deliverPreloadedFreightTime), // fix theta
            };
            deliverPreloadedFreightPath = new Path(new ArrayList<>(Arrays.asList(deliverPreloadedFreightWaypoints)));

        } else if (barcodeCase == 1) {
            deliverPreloadedFreightTime = 2.0;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(108, 50, 3*PI/4, 5, 30, 0, deliverPreloadedFreightTime), // fix theta
            };
            deliverPreloadedFreightPath = new Path(new ArrayList<>(Arrays.asList(deliverPreloadedFreightWaypoints)));
        } else {
            deliverPreloadedFreightTime = 2.25;
            Waypoint[] deliverPreloadedFreightWaypoints = new Waypoint[] {
                    new Waypoint(135, 36, PI, 30, 30, 0, 0),
                    new Waypoint(112, 53, 2*PI/3, 5, 30, 0, deliverPreloadedFreightTime), // fix theta
            };
            deliverPreloadedFreightPath = new Path(new ArrayList<>(Arrays.asList(deliverPreloadedFreightWaypoints)));
        }

        detector.setPipeline(Vision.Pipeline.AprilTag);

        ElapsedTime time = new ElapsedTime();
    }
}
