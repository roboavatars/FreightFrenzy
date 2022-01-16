package org.firstinspires.ftc.teamcode.RobotClasses.whitetapedetectionstuff;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@Config
public class ColorSensorForTapeDetection {
    public ColorSensor colorSensor;
    private boolean hitTape;
    public boolean isNull;

    double x,y,theta;

    public ColorSensorForTapeDetection(LinearOpMode op, String sensorName){
        colorSensor = op.hardwareMap.colorSensor.get(sensorName);
        isNull = true;
        hitTape = false;
    }

    public void update(double x, double y, double theta){
        if (white() && !hitTape){
            this.x = x;
            this.y = y;
            this.theta = theta;
            isNull = false;
            hitTape = true;
        } else if (!white()){
            hitTape = false;
        }
    }

    private boolean white(){
        return colorSensor.alpha()> Constants.COLOR_SENSOR_THRESHOLD;
    }
}
