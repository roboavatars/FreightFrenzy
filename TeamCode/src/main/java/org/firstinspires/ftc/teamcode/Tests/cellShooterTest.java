package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Cell Shooter Test")
@Config
public class cellShooterTest extends LinearOpMode{
    private DcMotorEx shooterTop;
    private DcMotorEx shooterBottom;

    private LinearOpMode op;

    public static double pTop = 0;
    public static double iTop = 0;
    public static double dTop = 0;
    public static double fTop = 0;

    public static double pBottom = 0;
    public static double iBottom = 0;
    public static double dBottom = 0;
    public static double fBottom = 0;

    public static int velo = 1000;
    public static boolean on = true;

    @Override
    public void runOpMode(){
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");

        shooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            shooterTop.setVelocityPIDFCoefficients(pTop,iTop,dTop,fTop);
            shooterBottom.setVelocityPIDFCoefficients(pBottom,iBottom,dBottom,fBottom);

            if (on){
                shooterTop.setVelocity(velo);
                shooterBottom.setVelocity(velo);
            } else{
                shooterTop.setPower(0);
                shooterBottom.setPower(0);
            }

            addPacket("Velocity Top", shooterTop.getVelocity());
            addPacket("Velocity Bottom", shooterBottom.getVelocity());
            addPacket("Target V", velo);

        }

    }

}
