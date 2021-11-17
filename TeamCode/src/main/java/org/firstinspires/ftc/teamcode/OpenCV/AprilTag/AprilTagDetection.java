package org.firstinspires.ftc.teamcode.OpenCV.AprilTag;

import org.opencv.core.Point;

public class AprilTagDetection
{
    public int id;
    public int hamming;
    public float decisionMargin;
    public Point center;
    public Point[] corners;
    public AprilTagPose pose;
}