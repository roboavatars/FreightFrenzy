package org.firstinspires.ftc.teamcode.Autonomous.Red;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodeDetector;
import org.firstinspires.ftc.teamcode.OpenCV.Barcode.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Pose;
import org.firstinspires.ftc.teamcode.Pathing.Target;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@Autonomous(name = "0 Red Auto Warehouse", preselectTeleOp = "2 Teleop 2P", group = "Red")
public class RedAutoWarehouse extends LinearOpMode {
    public static BarcodePipeline.Case barcodeCase;

    @Override
    public void runOpMode() {
        /*
        Timeline:
            detect barcode
            deliver preloaded freight on alliance shipping hub
            cycle
            park in warehouse
        */

        Robot robot = new Robot(this, 135, 78.5, 0, true, true, true);

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, true, true);
        barcodeDetector.start();

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

        double[] highCyclePos = new double[] {126, 74, 0.45};
        double[] midCyclePos = new double[] {127, 64, 0.3};
        double[] preloadDepositPos;

        int goToWarehouseSteps = 1;

        // Paths
        Path cycleScorePath = null;
        Path parkPath = null;

        int cycleCounter = 0;
        double passLineTime = 0;

        waitForStart();
        barcodeCase = barcodeDetector.getResult();

        ElapsedTime time = new ElapsedTime();

        if (barcodeCase == BarcodePipeline.Case.Left) {
            robot.cycleHub = Robot.DepositTarget.low;
            preloadDepositPos = new double[] {122, 69, 2*PI/15};
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.mid;
            preloadDepositPos = new double[] {124, 71, 2*PI/15};
        } else {
            robot.cycleHub = Robot.DepositTarget.high;
            preloadDepositPos = new double[] {130, 73, 2*PI/15};
        }

        Robot.log("Barcode Case: " + barcodeCase);
        try {
            barcodeDetector.stop();
        } catch (Exception ignore) {}

        Waypoint[] preloadScoreWaypoints = new Waypoint[]{
                new Waypoint(robot.x, robot.y, 3 * PI / 2, 10, 10, 0, 0),
                new Waypoint(preloadDepositPos[0], preloadDepositPos[1], preloadDepositPos[2] + PI, 2, -10, 0, preloadScoreTime),
        };
        Path preloadScorePath = new Path(preloadScoreWaypoints);

        robot.depositingFreight = true;
        robot.depositApproval = false;
        robot.depositState = 2;
        boolean intaked = false;

        while (opModeIsActive()) {
            addPacket("case", barcodeCase);

            addPacket("cycleCounter", cycleCounter);
            robot.intakeExtendDist = (int) Math.round(Constants.INTAKE_SLIDES_EXTEND_TICKS/3 + cycleCounter * Constants.INTAKE_SLIDES_EXTEND_TICKS/4);
            addPacket("w", robot.w);
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            if (!preloadScore) {
                //preload depositing
                addPacket("path", "initial deposit imo");

                Pose curPose = preloadScorePath.getRobotPose(Math.min(preloadScoreTime, time.seconds()));
                boolean thetaAtTarget = Math.abs(robot.theta - preloadDepositPos[2]) < PI/10;
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));
//                        .thetaKp(thetaAtTarget ? 1 : Drivetrain.thetaKp)
//                        .thetaKd(thetaAtTarget ? 0 : Drivetrain.thetaKd));

                robot.depositApproval = robot.isAtPose(preloadDepositPos[0], preloadDepositPos[1], preloadDepositPos[2], 2, 2, PI/20) && robot.notMoving();

                if (robot.depositState == 6) {
                    time.reset();
                    preloadScore = true;
                    robot.intakeApproval = true;
                }
            } else if (!goToWarehouse) {
                robot.depositApproval = false;

                if (timeLeft < parkThreshold && goToWarehouseSteps != 1) {
                    time.reset();
                    goToWarehouse = true;
                    cycleScore = true;
                } else {
                    switch (goToWarehouseSteps) {
                        case 1:
                            robot.drivetrain.constantStrafeConstant = 0; //-0.4
                            robot.setTargetPoint(new Target(141, 78, PI / 2).thetaKp((Math.abs(robot.theta - PI / 2) < PI / 6) ? Drivetrain.thetaKp : 10));
                            addPacket("path", "going to the wall right rn");
                            if (robot.x > 135.5 && Math.abs(PI / 2 - robot.theta) < PI / 10) {
                                goToWarehouseSteps++;
                                time.reset();
                            }
                            break;
                        case 2:
                            robot.drivetrain.setControls(0, -4, 0);
                            if (time.seconds() > 0.2) {
                                goToWarehouseSteps++;
                                robot.resetOdo(138, robot.y, PI/2);
                                time.reset();
                            }
                            break;
                        case 3:
                            robot.intake.setSlidesPosition((int) Math.round(robot.intakeExtendDist));
                            robot.drivetrain.constantStrafeConstant = 0;
//                            robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI / 2 > PI / 10 ? -0.5 : 0);
                            robot.setTargetPoint(new Target(robot.x, Robot.startIntakingRedAutoY + 5, PI/2 - PI/30).thetaKp(3));
                            passLineTime = time.seconds();
                            addPacket("path", "going to warehouse right rn");
                            if (robot.y > Robot.startIntakingRedAutoY - 1) goToWarehouseSteps++;
                            if (robot.intakeState == 3) {
                                robot.intakeApproval = false;
                                intaked = true;
                            }
                            break;
                        case 4:
                            robot.drivetrain.constantStrafeConstant = 0;
                            if (cycleCounter < 3) {
                                double y = Math.min(Robot.startIntakingRedAutoY/* + 0.75 * cycleCounter */+ 5 * (time.seconds() - passLineTime), 120);
                                double theta = PI / 2 - (PI / 10) * (Math.cos(4 * (time.seconds() - passLineTime)) - 1);
                                robot.setTargetPoint(new Target(136, y, theta));
                            } else {
                                double x = Math.max(138 - 1 * (time.seconds() - passLineTime), 130);
                                double y = Robot.startIntakingRedAutoY + 0.75 * (cycleCounter - 3) + (5 * Math.sin(4 * (time.seconds() - passLineTime)));
                                double theta = PI / 2 + (PI / 8 * Math.sin(4 * (time.seconds() - passLineTime)));
                                robot.setTargetPoint(new Target(x, y, theta));
                            }
                            if (robot.intakeState == 3 || intaked) {
                                robot.intakeApproval = false;
                                goToWarehouseSteps++;
                            }

                            addPacket("path", "creeping right rn");
                            break;
                        case 5:
                            robot.setTargetPoint(new Target(138, robot.startIntakingRedAutoY, PI / 2).thetaKp((Math.abs(robot.theta - PI / 2) < PI / 6) ? Drivetrain.thetaKp : 10));
                            if (robot.x > 135.5 && Math.abs(PI / 2 - robot.theta) < PI / 10) {
                                goToWarehouseSteps++;
                                time.reset();
                            }
                            break;
                        case 6:
                            robot.drivetrain.setControls(0, -4, 0);
                            if (time.seconds() > 0.2) {
                                goToWarehouseSteps++;
//                                robot.resetOdo(138, robot.y, PI/2);
                                time.reset();
                            }
                            break;
                        case 7:
                            goToWarehouseSteps = 1;

                            resetOdo = false;
                            robot.intakeUp = true;

                            Waypoint[] cycleScoreWaypoints;
//                            if (robot.cycleHub == Robot.DepositTarget.high) {
                                cycleScoreWaypoints = new Waypoint[]{
                                        new Waypoint(140, robot.y, 3 * PI / 2, 10, 10, 0, 0),
                                        new Waypoint(140, 72, 3 * PI / 2, 5, 1, 0, 0.75),
                                        new Waypoint(highCyclePos[0], highCyclePos[1], highCyclePos[2] + PI, 2, -10, 0, cycleScoreTime),
                                };
//                            } else {
//                                cycleScoreWaypoints = new Waypoint[]{
//                                        new Waypoint(140, robot.y, 3 * PI / 2, 10, 10, 0, 0),
//                                        new Waypoint(140, 79, 3 * PI / 2, 5, 1, 0, 0.75),
//                                        new Waypoint(midCyclePos[0], midCyclePos[1], midCyclePos[2] + PI, 2, -10, 0, cycleScoreTime),
//                                };
//                            }
                            cycleScorePath = new Path(cycleScoreWaypoints);

                            time.reset();
                            goToWarehouse = true;
                            break;
                    }
                }

                if (Math.abs(robot.y - 97) < 0.2 && !resetOdo) {
//                     robot.resetOdo(138, robot.y, PI/2);
                    resetOdo = true;
                }
            } else if (!cycleScore) {
                robot.drivetrain.constantStrafeConstant = robot.y > Robot.startIntakingRedAutoY ? -0.7 : 0;

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                boolean thetaAtTarget = Math.abs(robot.theta - preloadDepositPos[2]) < PI/10;
                robot.setTargetPoint(new Target(curPose)
                        .theta(robot.y > 92 ? PI/2 + PI/30 : curPose.theta + PI)
                        .thetaKp(robot.y > 92 ? 3 : Drivetrain.thetaKp));
//                        .thetaKp(thetaAtTarget ? 1 : Drivetrain.thetaKp)
//                        .thetaKd(thetaAtTarget ? 0 : Drivetrain.thetaKd)
//                        .xKp(Math.abs(robot.x - curPose.x) > 2 ? .25 : Drivetrain.xKp));
                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 97) < 0.5 && !resetOdo) {
//                    robot.resetOdo(138, robot.y, PI/2);
                    resetOdo = true;
                }

                robot.depositApproval = (robot.cycleHub == Robot.DepositTarget.high && robot.isAtPose(highCyclePos[0], highCyclePos[1], highCyclePos[2], 2, 2, PI/20))
//                        || (robot.cycleHub == Robot.DepositTarget.mid && robot.isAtPose(midCyclePos[0], midCyclePos[1], midCyclePos[2], 2, 2, PI/20))
                        && robot.notMoving();

                if (robot.depositState == 6) {
                    cycleCounter++;
                    highCyclePos[0] -= 0.55;
                    highCyclePos[2] += 0.03;
//                    if (cycleCounter == 2) robot.noExtend = false;

                    resetOdo = false;
                    goToWarehouse = false;
                    time.reset();
                    robot.intakeApproval = true;
                    robot.intakeUp = false;
                }
            } else { //parking
                robot.drivetrain.constantStrafeConstant = 0;
                robot.setTargetPoint(new Target(137.5, 111, PI/2));
                if (robot.intakeState == 6) robot.intakeEnabled = false;
                if (timeLeft < 1) {
                    robot.intakeEnabled = false;
                    robot.drivetrain.stop();
                }
                addPacket("path", "parking");
            }

            robot.update();
        }

        robot.stop();
//        try {
//            detector.stop();
//        } catch (Exception ignore) {}
    }
}
