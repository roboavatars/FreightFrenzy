package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SlowServo {
    private Servo servo;

    public double servoSpeed;
    public double targetPos;
    private double lastSetPos;

    private double lastUpdateTime;

    public SlowServo (LinearOpMode op, String servoName, double servoSpeed, double initPos){
        servo = op.hardwareMap.get(Servo.class, servoName);
        servo.setPosition(initPos);
        lastSetPos = initPos;
        this.servoSpeed = servoSpeed;
    }

    public void update (){
        double curTime = System.currentTimeMillis();

        if (targetPos - lastSetPos > 0){
            lastSetPos += (curTime - lastUpdateTime) * servoSpeed;
        } else if (targetPos - lastSetPos < 0){
            lastSetPos -= (curTime - lastUpdateTime) * servoSpeed;
        }

        lastSetPos = Math.max(Math.min(lastSetPos, 1), 0);
        servo.setPosition(lastSetPos);

        lastUpdateTime = curTime;
    }

    public void setPosition (double pos){
        targetPos = Math.max(Math.min(pos, 1), 0);
    }
}
