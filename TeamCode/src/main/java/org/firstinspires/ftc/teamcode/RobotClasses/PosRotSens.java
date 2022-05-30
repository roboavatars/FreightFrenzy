package org.firstinspires.ftc.teamcode.RobotClasses;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Localization.IMU;

public class PosRotSens {
    private IMU imu;
    private DistanceSensor dsens;
    private DistanceSensor X;
    private DistanceSensor Y;
    public PosRotSens (LinearOpMode op, boolean isRed) {
        if(isRed){
            X = op.hardwareMap.get(DistanceSensor.class, "DistanceSensorRedX");
        }
        else{
            X = op.hardwareMap.get(DistanceSensor.class, "DistanceSensorBlueX");
        }
        Y = op.hardwareMap.get(DistanceSensor.class, "DistanceSensorY");
        imu = new IMU(Math.PI / 2, op);
        double theta = imu.getTheta();
        //double distX = X.getDistance(DistanceUnit.INCH) + 6;
        //double distY = Y.getDistance(DistanceUnit.INCH) + 9;
    }
    public double[] update (){
        double notx = X.getDistance(DistanceUnit.INCH) + 6;
        double noty = Y.getDistance(DistanceUnit.INCH) + 9;
        double theta = imu.getTheta();
        double x = notx * Math.cos(theta-90);
        double y = noty * Math.sin(theta);
        double[] xytheta = new double[3];
        xytheta[0] = x;
        xytheta[1] = y;
        xytheta[2] = theta;
        return xytheta;
    }
}
