package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous(name = "0 Blue Auto Warehouse", preselectTeleOp = "2 Teleop 2P", group = "Red")
public class BlueAutoWarehouse extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase = BarcodePipeline.Case.Middle;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 9, 78.5, PI, true, false, true);

//        Vision detector = new Vision(this, true, Vision.Pipeline.Barcode);
//        detector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;
        boolean resetOdo = false;

        // Segment Times
        double cycleScoreTime = 1.5;
        double parkThreshold = 6;
        double preloadScoreTime = 1;

        double[] highCyclePos = new double[]{19, 73, PI - 0.3};
        double[] midCyclePos = new double[]{25, 72, PI - 0.3};
        double[] preloadDepositPos;

        int goToWarehouseSteps = 1;

        // Paths
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;
        double passLineTime = 0;

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.low;
            preloadDepositPos = new double[] {22, 69, 13*PI/15};
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.mid;
            preloadDepositPos = new double[] {20, 71, 13*PI/15};
        } else {
            robot.cycleHub = Robot.DepositTarget.high;
            preloadDepositPos = new double[] {18, 73, 13*PI/15};
        }

        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(robot.x, robot.y, 3 * PI / 2, 10, 10, 0, 0),
                new Waypoint(preloadDepositPos[0], preloadDepositPos[1], preloadDepositPos[2] + PI, 2, -10, 0, preloadScoreTime),
        };
        Path preloadScorePath = new Path(preloadScoreWaypoints);

        robot.depositingFreight = true;
        robot.depositApproval = false;
        robot.depositState = 2;

        while (opModeIsActive()) {
            addPacket("cycleCounter", cycleCounter);
            robot.intakeExtendDist = (int) Math.round(Constants.INTAKE_SLIDES_EXTEND_TICKS/3 + cycleCounter * Constants.INTAKE_SLIDES_EXTEND_TICKS/4);
            addPacket("w", robot.w);
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            if (!preloadScore) {
                addPacket("path", "initial deposit imo");

                Pose curPose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                robot.depositApproval = robot.isAtPose(preloadDepositPos[0], preloadDepositPos[1], preloadDepositPos[2], 2, 2, PI/10) && robot.notMoving();

                if (robot.depositState == 6) {
                    time.reset();
                    preloadScore = true;
                    robot.intakeApproval = true;
                }
            } else if (!goToWarehouse) {
                robot.depositApproval = false;
                if (robot.depositState != 1 && robot.depositState != 7) robot.intakeApproval = false;

                switch (goToWarehouseSteps) {
                    case 1:
                        robot.drivetrain.constantStrafeConstant = 0; //0.4
                        robot.setTargetPoint(new Target(3, 78, PI/2).thetaKp((Math.abs(robot.theta - PI/2) < PI/6) ? Drivetrain.thetaKp : 10));
                        addPacket("path", "going to the wall right rn");
                        if (robot.x < 9 && Math.abs(PI / 2 - robot.theta) < PI / 10)
                            goToWarehouseSteps++;
                        break;
                    case 2:
                        robot.intake.setSlidesPosition((int) Math.round(robot.intakeExtendDist));
                        robot.drivetrain.constantStrafeConstant = 0;
                        robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI/2 < -PI / 10 ? 0.5 : 0);
                        passLineTime = time.seconds();
                        addPacket("path", "going to warehouse right rn");
                        if (robot.y > Robot.startIntakingAutoY-1) goToWarehouseSteps++;
                        break;
                    case 3:
                        if (timeLeft > parkThreshold) {
                            robot.drivetrain.constantStrafeConstant = 0;
                            if (cycleCounter < 3) {
                                double y = Math.min(Robot.startIntakingAutoY + 0.75 * cycleCounter + 3 * (time.seconds() - passLineTime), 121);
                                double theta = PI/2 + (PI/15)*(Math.cos(4*(time.seconds() - passLineTime))-1);
                                robot.setTargetPoint(new Target(6, y, theta));
                            } else {
                                double x = Math.min(6 + 1 * (time.seconds() - passLineTime), 14);
                                double y = Robot.startIntakingAutoY + 0.75 * (cycleCounter - 3) + (5*Math.sin(4*(time.seconds() - passLineTime)));
                                double theta = PI/2 - (PI/8 * Math.sin(4*(time.seconds() - passLineTime)));
                                robot.setTargetPoint(new Target(x, y, theta));
                            }


                            addPacket("path", "creeping right rn");
                        } else {
                            robot.depositEnabled = false;
                            robot.setTargetPoint(new Target(6.5, 112, PI / 2));
                            addPacket("path", "going to park right rn");
                        }

                        if (robot.intakeState == 3) goToWarehouseSteps++;
                        break;
                    case 4:
                        robot.setTargetPoint(new Target(3, robot.startIntakingAutoY, PI/2).thetaKp((Math.abs(robot.theta-PI/2) < PI/6) ? Drivetrain.thetaKp : 10));
                        if (robot.x < 7 && Math.abs(PI/2 - robot.theta) < PI/10) goToWarehouseSteps++;
                        break;
                    case 5:
                        if (timeLeft < parkThreshold && robot.y > 112) {
                            time.reset();
                            goToWarehouse = true;
                            cycleScore = true;
                        } else {
                            goToWarehouseSteps = 1;

                            resetOdo = false;

                            Waypoint[] cycleScoreWaypoints;
                            if (robot.cycleHub == Robot.DepositTarget.high) {
                                cycleScoreWaypoints = new Waypoint[] {
                                        new Waypoint(4, robot.y, 3*PI/2, 10, 10, 0, 0),
                                        new Waypoint(4, 79, 3*PI/2, 5, 1, 0, 0.75),
                                        new Waypoint(highCyclePos[0], highCyclePos[1], highCyclePos[2] + PI, 2, -10, 0, cycleScoreTime),
                                };
                            } else {
                                cycleScoreWaypoints = new Waypoint[] {
                                        new Waypoint(4, robot.y, 3*PI/2, 10, 10, 0, 0),
                                        new Waypoint(4, 79, 3*PI/2, 5, 1, 0, 0.75),
                                        new Waypoint(midCyclePos[0], midCyclePos[1], midCyclePos[2] + PI, 2, -10, 0, cycleScoreTime),
                                };
                            }
                            cycleScorePath = new Path(cycleScoreWaypoints);

                            time.reset();
                            goToWarehouse = true;
                        }
                        break;
                }

                if (Math.abs(robot.y - 97) < 0.5 && !resetOdo) {
                     robot.resetOdo(6, robot.y, PI/2);
                    resetOdo = true;
                }
            } else if (!cycleScore) {
                robot.drivetrain.constantStrafeConstant = robot.y > Robot.startIntakingAutoY ? 0.7 : 0;

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));

                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 97) < 0.5 && !resetOdo) {
                    robot.resetOdo(6, robot.y, PI/2);
                    resetOdo = true;
                }

                robot.depositApproval = (robot.cycleHub == Robot.DepositTarget.high && robot.isAtPose(highCyclePos[0], highCyclePos[1], highCyclePos[2], 2, 2, PI/10))
                        || (robot.cycleHub == Robot.DepositTarget.mid && robot.isAtPose(midCyclePos[0], midCyclePos[1], midCyclePos[2], 2, 2, PI/10))
                        && robot.notMoving();

                if (robot.depositState == 6) {
                    cycleCounter++;
                    highCyclePos[0] += 0.5;
                    highCyclePos[2] -= 0.025;
//                    if (cycleCounter == 2) robot.noExtend = false;

                    resetOdo = false;
                    goToWarehouse = false;
                    time.reset();
                    robot.intakeApproval = true;
                }
            } else {
                if (robot.depositState == 2) robot.intakeApproval = false;
                robot.depositEnabled = false;
                if (timeLeft < 2) {
                    robot.intakeOff = true;
                    robot.intakeApproval = false;
                }
                robot.drivetrain.stop();
                addPacket("path", "stopped rn");
            }

            robot.update();
        }

        robot.stop();
//        try {
//            detector.stop();
//        } catch (Exception ignore) {}
    }
}
