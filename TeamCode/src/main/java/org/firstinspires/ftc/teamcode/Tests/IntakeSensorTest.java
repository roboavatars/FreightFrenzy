package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp
public class IntakeSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");

        waitForStart();
        while (opModeIsActive()) {
            addPacket("distance", intakeSensor.getDistance(DistanceUnit.MM));
            addPacket("intaked", intakeSensor.getDistance(DistanceUnit.MM) < Constants.INTAKE_DISTANCE_THRESHOLD);
            sendPacket();
        }
    }
}
