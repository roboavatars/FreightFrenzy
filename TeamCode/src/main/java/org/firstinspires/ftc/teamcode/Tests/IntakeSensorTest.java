package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@TeleOp(name = "Intake Sensor Test")
public class IntakeSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");

        waitForStart();

        while (opModeIsActive()) {
            addPacket("Measured Distance", intakeSensor.getDistance(DistanceUnit.MM));
            addPacket("Threshold Distance", Constants.INTAKE_DISTANCE_THRESHOLD);
            addPacket("intakeFull", intakeSensor.getDistance(DistanceUnit.MM) < Constants.INTAKE_DISTANCE_THRESHOLD);
            addPacket("red", intakeSensor.red());
            addPacket("green", intakeSensor.green());
            addPacket("blue", intakeSensor.blue());
            addPacket("alpha", intakeSensor.alpha());
            sendPacket();

            telemetry.addData("Measured Distance", intakeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Threshold Distance", Constants.INTAKE_DISTANCE_THRESHOLD);
            telemetry.addData("intakeFull", intakeSensor.getDistance(DistanceUnit.MM) < Constants.INTAKE_DISTANCE_THRESHOLD);
            telemetry.addData("red", intakeSensor.red());
            telemetry.addData("green", intakeSensor.green());
            telemetry.addData("blue", intakeSensor.blue());
            telemetry.addData("alpha", intakeSensor.alpha());
            telemetry.update();
        }
    }
}
