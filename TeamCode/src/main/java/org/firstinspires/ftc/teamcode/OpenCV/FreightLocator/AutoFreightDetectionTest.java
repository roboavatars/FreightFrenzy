package org.firstinspires.ftc.teamcode.OpenCV.FreightLocator;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawFreight;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawPoint;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

@TeleOp(name = "Auto Freight Detection Test")
@Disabled
public class AutoFreightDetectionTest extends LinearOpMode {

    private Robot robot;
    private FreightLocator locator;

    private ArrayList<Freight> freights;
    private ArrayList<Waypoint> freightWaypoints;
    private double[] freightPos;
    private double[] freightIntakeTheta = new double[3];
    private double freightTime = 0;
    private Path freightPath;

    private double intakePower = 0;
    private boolean start = false;
    private double startTime, driveTime;

    private double x, y, theta;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 138, 81, PI / 2, false, true);
        locator = new FreightLocator(this);
        locator.start();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            x = robot.x;
            y = robot.y;
            theta = robot.theta;

            // Get Freight Positions
            freights = locator.getFreight(x, y, theta);
            for (int i = 0; i < freights.size(); i++) {
                if (i == 0) {
                    drawFreight(freights.get(i), "yellow");
                }
            }

            if (start) {
                // Follow Path
                double curTime = Math.min(timer.seconds(), freightTime);
                if (freights.size() >= 1 && curTime < 0.75) {
                    robot.setTargetPoint(freightPath.getRobotPose(curTime));
                } else if (freights.size() >= 1 && curTime < 1.5) {
                    robot.setTargetPoint(new Target(freightPath.getRobotPose(curTime)).thetaW0(freightIntakeTheta[0]));
                } else {
                    robot.setTargetPoint(new Target(freightPath.getRobotPose(curTime)).thetaW0(PI / 2));
                }

                intakePower = 1;
                driveTime = (double) System.currentTimeMillis() / 1000 - startTime;

                // Stop Following if Done
                if (curTime == freightTime && (robot.notMoving() || robot.isAtPose(111, 63, PI / 2))) {
                    start = false;
                    intakePower = 0;
                }
            } else {
                // Intake Controls
                if (gamepad1.right_trigger > 0) {
                    intakePower = 1;
                } else if (gamepad1.left_trigger > 0) {
                    intakePower = -1;
                } else {
                    intakePower = 0;
                }

                // Drivetrain Controls
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

                // Reset Odo
                if (gamepad1.x) {
                    robot.resetOdo(138, 81, PI / 2);
                }

                // Start Path Following
                if (gamepad1.a) {
                    start = true;
                    startTime = (double) System.currentTimeMillis() / 1000;
                    timer.reset();
                }

                // Generate Path
                freightWaypoints = new ArrayList<>();
                freightWaypoints.add(new Waypoint(x, y, theta, 50, 60, 0, 0));

                freightTime = 0;
                if (freights.size() >= 1) {
                    freightPos = freights.get(0).driveToFreight(x, y);
                    freightTime += 1.5;
                    if (freightPos[1] > 130) {
                        freightPos[2] = 0;
                        freightIntakeTheta[0] = freightPos[0] - x < 0 ? 3 * PI / 4 : PI / 4;
                    } else {
                        freightIntakeTheta[0] = freightPos[2];
                    }
                    freightWaypoints.add(new Waypoint(freightPos[0], Math.min(130, freightPos[1]), freightIntakeTheta[0], 30, 10, 0, freightTime));
                }

                freightTime += 1.5;
                freightWaypoints.add(new Waypoint(138, 81, PI / 2, -30, -60, 0, freightTime));
                freightPath = new Path(freightWaypoints);
            }

            // Force Stop
            if (gamepad1.b) {
                start = false;
                intakePower = 0;
                robot.drivetrain.stop();
            }

            // Draw Path
            for (double i = (start ? driveTime : 0); i < freightTime; i += freightTime / 50) {
                try {
                    drawPoint(freightPath.getRobotPose(i).x, freightPath.getRobotPose(i).y, "blue");
                } catch (ArrayIndexOutOfBoundsException ignore) {
                }
            }

            addPacket("start", start);
            robot.intake.setPower(intakePower);
            robot.update();
        }

        locator.stop();
        robot.stop();
    }
}
