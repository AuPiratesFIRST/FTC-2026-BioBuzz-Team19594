package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import org.opencv.core.Point;

public class CameraCoordinateTransformer {
    // Mounting Constants
    public static double CAM_HEIGHT_Z = 12.0;
    public static double CAM_PITCH_DEGREES = 25.0;
    public static double CAM_OFFSET_FORWARD = 6.0;

    // FOV Constants
    public static double HORIZONTAL_FOV = 78.0;
    public static double VERTICAL_FOV = 43.3;
    public static double RES_W = 640;
    public static double RES_H = 480;

    public static Pose projectPixelToField(Point pixel, Pose robotPose) {
        double dAngleX = (pixel.x - (RES_W / 2.0)) * (HORIZONTAL_FOV / RES_W);
        double dAngleY = ((RES_H / 2.0) - pixel.y) * (VERTICAL_FOV / RES_H);

        double totalPitchRad = Math.toRadians(CAM_PITCH_DEGREES + dAngleY);
        double groundDist = CAM_HEIGHT_Z / Math.tan(totalPitchRad);

        double relX = groundDist + CAM_OFFSET_FORWARD;
        double relY = groundDist * Math.tan(Math.toRadians(dAngleX));

        double heading = robotPose.heading();
        double fieldX = robotPose.x() + (relX * Math.cos(heading) - relY * Math.sin(heading));
        double fieldY = robotPose.y() + (relX * Math.sin(heading) + relY * Math.cos(heading));

        return new Pose(fieldX, fieldY, heading);
    }
}