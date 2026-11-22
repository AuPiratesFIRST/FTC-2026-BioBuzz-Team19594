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
        if (pixel == null || robotPose == null) return null;

        double dAngleX = (pixel.x - (RES_W / 2.0)) * (HORIZONTAL_FOV / RES_W);
        double dAngleY = ((RES_H / 2.0) - pixel.y) * (VERTICAL_FOV / RES_H);

        double totalPitchRad = Math.toRadians(CAM_PITCH_DEGREES + dAngleY);

        // Bounds check: pitch near or crossing zero (near-parallel or above horizon) produces division by zero/huge distances
        if (totalPitchRad <= Math.toRadians(1.0)) {
            return null;
        }

        double tanPitch = Math.tan(totalPitchRad);
        if (tanPitch <= 0.01) {
            return null;
        }

        double groundDist = CAM_HEIGHT_Z / tanPitch;
        if (Double.isNaN(groundDist) || Double.isInfinite(groundDist) || groundDist > 300.0) {
            return null;
        }

        double relX = groundDist + CAM_OFFSET_FORWARD;
        double relY = groundDist * Math.tan(Math.toRadians(dAngleX));

        double heading = robotPose.heading();
        double fieldX = robotPose.x() + (relX * Math.cos(heading) - relY * Math.sin(heading));
        double fieldY = robotPose.y() + (relX * Math.sin(heading) + relY * Math.cos(heading));

        return new Pose(fieldX, fieldY, heading);
    }
}