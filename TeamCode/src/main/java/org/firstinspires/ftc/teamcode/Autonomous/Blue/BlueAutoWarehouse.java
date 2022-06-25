package org.firstinspires.ftc.teamcode.Autonomous.Blue;

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
@Autonomous(name = "0 Blue Auto Warehouse", preselectTeleOp = "2 Teleop 2P", group = "Blue")
public class BlueAutoWarehouse extends LinearOpMode {
    public BarcodePipeline.Case barcodeCase;
    public static double xDrift = 0;

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

        BarcodeDetector barcodeDetector = new BarcodeDetector(this, false, true);
        barcodeDetector.start();

        // Segments
        boolean preloadScore = false;
        boolean goToWarehouse = false;
        boolean cycleScore = false;
        boolean park = false;
        boolean resetOdo = false;

        // Segment Times
        double cycleScoreTime = 2;
        double parkThreshold = 4.5;
        double preloadScoreTime = 1;

        double[] highCyclePos = new double[]{18, 72, PI - 0.35};
        double[] midCyclePos = new double[]{25, 72, PI - 0.3};
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
            preloadDepositPos = new double[]{23, 69, 13 * PI / 15};
        } else if (barcodeCase == BarcodePipeline.Case.Middle) {
            robot.cycleHub = Robot.DepositTarget.mid;
            preloadDepositPos = new double[]{21, 71, 13 * PI / 15};
        } else {
            robot.cycleHub = Robot.DepositTarget.high;
            preloadDepositPos = new double[]{14, 74, 13 * PI / 15};
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
            addPacket("barcode", barcodeCase);

            addPacket("cycleCounter", cycleCounter);
            robot.intakeExtendDist = (int) Math.round(Constants.INTAKE_SLIDES_EXTEND_TICKS/3 + (cycleCounter + 1) * Constants.INTAKE_SLIDES_EXTEND_TICKS/4);
            addPacket("w", robot.w);
            double timeLeft = 30 - (System.currentTimeMillis() - robot.startTime) / 1000;
            addPacket("time left", timeLeft);

            if (!preloadScore) {
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

                if (timeLeft < parkThreshold && goToWarehouseSteps != 1 && goToWarehouseSteps != 2) {
                    time.reset();
                    goToWarehouse = true;
                    cycleScore = true;
                } else {
                    addPacket("go to warehouse step", goToWarehouseSteps);
                    switch (goToWarehouseSteps) {
                        case 1:
                            robot.drivetrain.constantStrafeConstant = 0; //0.4
                            robot.setTargetPoint(new Target(3, 73, PI / 2).thetaKp((Math.abs(robot.theta - PI / 2) < PI / 6) ? Drivetrain.thetaKp : 10));
                            addPacket("path", "going to the wall right rn");
                            if (robot.x < (8.5 + xDrift*cycleCounter) && Math.abs(PI / 2 - robot.theta) < PI / 10) {
                                goToWarehouseSteps++;
                                time.reset();
                            }
                            break;
                        case 2:
                            robot.drivetrain.setControls(0, 4, 0);
                            if (time.seconds() > (0.3 + 0.05 * cycleCounter)) {
                                goToWarehouseSteps++;
                                robot.resetOdo(6, robot.y, PI/2);
                                time.reset();
                            }
                            break;
                        case 3:
                            robot.intake.setSlidesPosition((int) Math.round(robot.intakeExtendDist));
                            robot.drivetrain.constantStrafeConstant = 0.3;
//                            robot.drivetrain.setGlobalControls(0, 0.7, robot.theta - PI / 2 > PI / 10 ? -0.5 : 0);
                            robot.setTargetPoint(new Target(robot.x, Robot.startIntakingBlueAutoY, PI/2 + PI/20).thetaKp(3));
                            passLineTime = time.seconds();
                            addPacket("path", "going to warehouse right rn");
                            if (robot.y > Robot.startIntakingBlueAutoY - 1) goToWarehouseSteps++;
                            if (robot.intakeState == 3) {
                                robot.intakeApproval = false;
                                intaked = true;
                            }
                            break;
                        case 4:
                            robot.drivetrain.constantStrafeConstant = 0;
                            if (robot.antiStallStep == "Reverse Intake" && robot.intakeState == 2) {
                                robot.setTargetPoint(6.5, Robot.startIntakingBlueAutoY, PI/2);
                            } else {
                                if (cycleCounter < 3) {
                                    double y = Math.min(Robot.startIntakingBlueAutoY/* + 0.75 * cycleCounter */+ 5 * (time.seconds() - passLineTime), 121);
                                    double theta = PI / 2 + (PI / 15) * (Math.cos(4 * (time.seconds() - passLineTime)) - 1);
                                    robot.setTargetPoint(new Target(10, y, theta));
                                } else {
                                    double x = Math.min(6 + 1 * (time.seconds() - passLineTime), 14);
                                    double y = Robot.startIntakingBlueAutoY + 3 * (cycleCounter - 3) + (5 * Math.sin(4 * (time.seconds() - passLineTime)));
                                    double theta = PI / 2 - (PI / 8 * Math.sin(4 * (time.seconds() - passLineTime)));
                                    robot.setTargetPoint(new Target(x, y, theta));
                                }
                                if (robot.intakeState == 3 || robot.intakeState == 4) {
                                    robot.intakeApproval = false;
                                    goToWarehouseSteps++;
                                }
                            }
                            addPacket("path", "creeping right rn");
                            break;
                        case 5:
//                            if (cycleCounter < 2) {
                                robot.setTargetPoint(new Target(6, Robot.startIntakingBlueAutoY, PI / 2).thetaKp((Math.abs(robot.theta - PI / 2) < PI / 6) ? Drivetrain.thetaKp : 10));
//                            } else {
//                                robot.setTargetPoint(new Target(9, Robot.startIntakingBlueAutoY, PI / 2).thetaKp((Math.abs(robot.theta - PI / 2) < PI / 6) ? Drivetrain.thetaKp : 10));
//                            }
                            if (robot.x < (10 + xDrift*cycleCounter) && Math.abs(PI / 2 - robot.theta) < PI / 10) {
                                goToWarehouseSteps++;
                                time.reset();
                            }
                            break;
                        case 6:
                            robot.drivetrain.setControls(0, 4, 0);
                            if (time.seconds() > 0.3) {
                                goToWarehouseSteps++;
//                                robot.resetOdo(6, robot.y, PI/2);
                                time.reset();
                            }
                            break;
                        case 7:
                            goToWarehouseSteps = 1;

                            resetOdo = false;
                            robot.intakeUp = true;

                            Waypoint[] cycleScoreWaypoints;
                            if (robot.cycleHub == Robot.DepositTarget.high) {
                                cycleScoreWaypoints = new Waypoint[]{
                                        new Waypoint(4, robot.y, 3 * PI / 2, 10, 10, 0, 0),
                                        new Waypoint(4.5, 78, 3 * PI / 2, 5, 1, 2, 1.25),
                                        new Waypoint(highCyclePos[0], highCyclePos[1], highCyclePos[2] + PI, 1, -10, 0, cycleScoreTime),
                                };
                            } else {
                                cycleScoreWaypoints = new Waypoint[]{
                                        new Waypoint(4, robot.y, 3 * PI / 2, 10, 10, 0, 0),
                                        new Waypoint(4, 79, 3 * PI / 2, 5, 1, 0, 0.75),
                                        new Waypoint(midCyclePos[0], midCyclePos[1], midCyclePos[2] + PI, 2, -10, 0, cycleScoreTime),
                                };
                            }
                            cycleScorePath = new Path(cycleScoreWaypoints);

                            time.reset();
                            goToWarehouse = true;
                            break;
                    }
                }

                if (Math.abs(robot.y - 90) < 0.5 && !resetOdo) {
//                    robot.resetOdo(6, robot.y, PI / 2);
                    resetOdo = true;
                }
            } else if (!cycleScore) {
                robot.drivetrain.constantStrafeConstant = robot.y > Robot.startIntakingBlueAutoY ? 0.7 : 0;

                if (robot.deposit.getSlidesPos() < 250 && !robot.deposit.slidesisHome()) robot.intake.setSlidesPosition(150);
                else robot.intake.setSlidesPosition(Constants.INTAKE_SLIDES_HOME_TICKS);

                Pose curPose = cycleScorePath.getRobotPose(Math.min(cycleScoreTime, time.seconds()));
                boolean thetaAtTarget = Math.abs(robot.theta - preloadDepositPos[2]) < PI/20;
                robot.setTargetPoint(new Target(curPose).theta(curPose.theta + PI));
//                        .thetaKp(thetaAtTarget ? 1 : Drivetrain.thetaKp)
//                        .thetaKd(thetaAtTarget ? 0 : Drivetrain.thetaKd));
                addPacket("path", "going to deposit right rn");

                if (Math.abs(robot.y - 90) < 0.5 && !resetOdo) {
//                    robot.resetOdo(6, robot.y, PI / 2);
                    resetOdo = true;
                }

                robot.depositApproval = robot.isAtPose(highCyclePos[0], highCyclePos[1], highCyclePos[2], 2, 2, PI/20)
//                        || (robot.cycleHub == Robot.DepositTarget.mid && robot.isAtPose(midCyclePos[0], midCyclePos[1], midCyclePos[2], 2, 2, PI/20))
                        && robot.notMoving();

                if (robot.depositState == 6) {
                    cycleCounter++;
                    highCyclePos[0] += 0.7;
                    highCyclePos[2] -= 0.035;
//                    if (cycleCounter == 2) robot.noExtend = false;

                    resetOdo = false;
                    goToWarehouse = false;
                    time.reset();
                    robot.intakeApproval = true;
                    robot.intakeUp = false;
                }
            } else { //parking
                robot.intakeExtendDist = Constants.INTAKE_SLIDES_HOME_TICKS;
                robot.drivetrain.constantStrafeConstant = 0;
                if (!robot.isAtPose(6.5, 112, PI/2)) robot.setTargetPoint(new Target(6.5, 108, PI / 2));
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
