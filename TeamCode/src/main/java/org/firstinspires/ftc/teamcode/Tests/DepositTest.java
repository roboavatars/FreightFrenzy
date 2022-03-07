package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Deposit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret;

@TeleOp
@Config
public class DepositTest extends LinearOpMode {

   public static boolean home = true;
   public static double theta = Constants.TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA;

   @Override
   public void runOpMode(){
      Deposit deposit = new Deposit(this, false);
      Turret turret = new Turret(this, false, PI/2);

      waitForStart();
      while (opModeIsActive()) {
         if (home) {
            deposit.setDepositHome();
            turret.setHome();
         } else {
            deposit.setDepositControls(Robot.DepositTarget.allianceHigh, Constants.SLIDES_DISTANCE_HIGH);
            turret.setDepositing(theta*PI);
         }
         deposit.update(false, turret.isHome());
         turret.update(deposit.armHome());

         addPacket("arm", deposit.getArmPosition());
         addPacket("slides", deposit.getSlidesPosition());
         sendPacket();
      }
   }
}
