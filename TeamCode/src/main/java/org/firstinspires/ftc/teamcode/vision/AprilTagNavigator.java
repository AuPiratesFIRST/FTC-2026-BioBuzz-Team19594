package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drive.TileCoordinate;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Optimized AprilTag Navigator for BIOBUZZ.
 * Evaluates camera localization directly within Pedro Pathing Coordinates (0-144 inches).
 * Polymorphically handles both AprilTagSingleDetection and AprilTagClusterDetection.
 */
public class AprilTagNavigator {

    private final DriveSubsystem driveSubsystem;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;

    private final double MIN_DETECTION_DISTANCE = 1.0;
    private final double MAX_DETECTION_DISTANCE = 250;
    private final double MIN_DECISION_MARGIN = 0.15;

    private static final double CAMERA_HEIGHT = 9.375;
    private static final double CAMERA_Y_OFFSET = 9.0; // inches forward of center along robot's forward vector

    private final double MAX_TRUST_DISTANCE = 100.0;
    private final double MIN_TRUST_DISTANCE = 6.0;
    private final double TRUST_WEIGHT_BASE = 0.1;
    private int cameraExposureMs = 14;
    private int cameraGain = 35;
    private boolean cameraControlsSet = false;

    private double lastErrorX = 0;
    private double lastTime = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();

    public enum HiveState {
        RED_TILTED_UP(+30.0),
        BLUE_TILTED_UP(-30.0);

        public final double pitchAngleDegrees;
        HiveState(double pitchAngleDegrees) {
            this.pitchAngleDegrees = pitchAngleDegrees;
        }
    }

    private HiveState currentHiveState = HiveState.RED_TILTED_UP;

    public AprilTagNavigator(DriveSubsystem driveSubsystem, AprilTagProcessor aprilTag,
                             VisionPortal visionPortal, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.aprilTag = aprilTag;
        this.visionPortal = visionPortal;
        this.telemetry = telemetry;
    }

    public Pose getPedroPose() {
        if (driveSubsystem == null) return new Pose(0, 0, 0);
        TileCoordinate pos = driveSubsystem.getCurrentPosition();
        double heading = driveSubsystem.getHeading();
        return new Pose(pos.getX(), pos.getY(), heading);
    }

    public void setHiveState(HiveState state) {
        this.currentHiveState = state;
    }

    public HiveState getHiveState() {
        return currentHiveState;
    }

    /**
     * Dynamic detection: calculates tag Z-height to determine if the target side
     * is Tilted Up (56.25" centerline) or Tilted Down (28.25" centerline).
     */
    public HiveState detectHiveState() {
        AprilTagDetection detection = getBestDetection();
        if (detection == null || detection.ftcPose == null) {
            return currentHiveState;
        }

        double elevationRad = Math.toRadians(-detection.ftcPose.pitch);
        double detectedZ = CAMERA_HEIGHT + (detection.ftcPose.range * Math.sin(elevationRad));

        if (detectedZ > 42.05) {
            currentHiveState = HiveState.RED_TILTED_UP;
        } else {
            currentHiveState = HiveState.BLUE_TILTED_UP;
        }

        return currentHiveState;
    }

    public int getTagId(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            return ((AprilTagSingleDetection) detection).id;
        }
        return -1;
    }

    public boolean isHiveTag(AprilTagDetection detection) {
        if (detection == null) return false;

        if (detection instanceof AprilTagClusterDetection) {
            return true;
        } else if (detection instanceof AprilTagSingleDetection) {
            int id = ((AprilTagSingleDetection) detection).id;
            return (id >= 30 && id <= 45) || (id >= 4 && id <= 7);
        }
        return false;
    }

    public boolean updateGlobalPoseEstimate() {
        if (driveSubsystem != null && Math.abs(driveSubsystem.getAngularVelocity()) > 2.0) {
            return false;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return false;

        double weightedX = 0, weightedY = 0, weightedSin = 0, weightedCos = 0;
        double totalWeight = 0;

        for (AprilTagDetection det : detections) {
            if (det.ftcPose == null) continue;

            double dist = det.ftcPose.range;
            if (dist > MAX_TRUST_DISTANCE || dist < MIN_TRUST_DISTANCE) continue;

            double weight = (1.0 - (dist / MAX_TRUST_DISTANCE));

            if (det instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) det;
                weight *= (cluster.percentClusterFound / 100.0);
            } else if (det instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) det;
                weight *= (single.decisionMargin / 30.0);
            }

            weight = Range.clip(weight, 0.0, 1.0);

            double[] pose = calculateRobotPoseWithoutUpdating(det);
            if (pose == null) continue;

            weightedX += pose[0] * weight;
            weightedY += pose[1] * weight;
            weightedSin += Math.sin(pose[2]) * weight;
            weightedCos += Math.cos(pose[2]) * weight;
            totalWeight += weight;
        }

        if (totalWeight < 0.1) return false;

        double finalX = weightedX / totalWeight;
        double finalY = weightedY / totalWeight;
        double finalHeading = Math.atan2(weightedSin / totalWeight, weightedCos / totalWeight);

        if (driveSubsystem != null) {
            TileCoordinate currentPos = driveSubsystem.getCurrentPosition();
            double error = currentPos.distanceTo(new TileCoordinate(finalX, finalY));

            if (error > 12.0) {
                driveSubsystem.setPosition(new TileCoordinate(finalX, finalY));
                driveSubsystem.setHeading(finalHeading);
                telemetry.addData("Vision", "POS SNAP: Correction %.1f in", error);
            } else {
                double newX = currentPos.getX() + (finalX - currentPos.getX()) * TRUST_WEIGHT_BASE;
                double newY = currentPos.getY() + (finalY - currentPos.getY()) * TRUST_WEIGHT_BASE;
                driveSubsystem.setPosition(new TileCoordinate(newX, newY));
                driveSubsystem.setHeading(finalHeading);
            }
        }
        return true;
    }

    /**
     * Calculates the estimated Pedro Pathing pose from a raw AprilTag detection.
     * Evaluates completely inside the Pedro coordinate frame.
     *
     * @return double array containing {PedroX, PedroY, PedroHeadingRadians}
     */
    private double[] calculateRobotPoseWithoutUpdating(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) return null;

        double tagPedroX = 0, tagPedroY = 0, tagZ = 0, tagPedroHeadingDeg = 0;

        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null) return null;

            VectorF fieldPos = cluster.metadata.fieldPosition;
            if (fieldPos != null && (fieldPos.get(0) != 0 || fieldPos.get(1) != 0)) {
                tagPedroX = fieldPos.get(0);
                tagPedroY = fieldPos.get(1);
                tagZ = fieldPos.get(2);
                tagPedroHeadingDeg = 0;
            } else {
                // Pass currentHiveState into the position lookup
                double[] clusterPos = FieldConstants.getClusterPosition(cluster.metadata.shortName, currentHiveState);
                if (clusterPos == null) return null;

                tagPedroX = clusterPos[0];
                tagPedroY = clusterPos[1];
                tagZ = clusterPos[2];
                tagPedroHeadingDeg = clusterPos[3];
            }
        } else if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            double[] tagPos = null;
            for (double[] p : FieldConstants.APRILTAG_POSITIONS) {
                if (p != null && (int) p[0] == single.id) {
                    tagPos = p;
                    break;
                }
            }
            if (tagPos == null) return null;

            tagPedroX = tagPos[1];
            tagPedroY = tagPos[2];
            tagPedroHeadingDeg = tagPos[3];
            tagZ = (tagPos.length > 4) ? tagPos[4] : 0.0;
        }

        // Dynamic local pivot origin based on cluster side
        if (isHiveTag(detection)) {
            double tiltRad = Math.toRadians(currentHiveState.pitchAngleDegrees);

            // Use the cluster's native Y-coordinate (12.0 or 132.0) as the local pivot axis center
            double clusterPivotY = tagPedroY;
            double dY = tagPedroY - clusterPivotY;
            double dZ = tagZ - FieldConstants.HIVE_PIVOT_HEIGHT_INCHES;

            tagPedroY = clusterPivotY + dY * Math.cos(tiltRad) - dZ * Math.sin(tiltRad);
            tagZ = FieldConstants.HIVE_PIVOT_HEIGHT_INCHES + dY * Math.sin(tiltRad) + dZ * Math.cos(tiltRad);
        }

        // 1. Calculate true global Pedro heading from tag orientation and yaw
        double pedroHeadingRad = AngleUnit.normalizeRadians(
                Math.toRadians(tagPedroHeadingDeg) + Math.PI - Math.toRadians(detection.ftcPose.yaw)
        );

        // 2. Map local camera frame (x=right, y=forward) into global Pedro field coordinates
        double localX = detection.ftcPose.x; // lateral translation
        double localY = detection.ftcPose.y; // forward depth

        double deltaPedroX = localY * Math.cos(pedroHeadingRad) - localX * Math.sin(pedroHeadingRad);
        double deltaPedroY = localY * Math.sin(pedroHeadingRad) + localX * Math.cos(pedroHeadingRad);

        // 3. Subtract vector from tag center to get camera position
        double cameraPedroX = tagPedroX - deltaPedroX;
        double cameraPedroY = tagPedroY - deltaPedroY;

        // 4. Offset back to robot center
        double finalPedroX = cameraPedroX - CAMERA_Y_OFFSET * Math.cos(pedroHeadingRad);
        double finalPedroY = cameraPedroY - CAMERA_Y_OFFSET * Math.sin(pedroHeadingRad);

        return new double[]{finalPedroX, finalPedroY, pedroHeadingRad};
    }

    public AprilTagDetection getBestDetectionForTag(int tagId) {
        return aprilTag.getDetections().stream()
                .filter(tag -> tag.ftcPose != null && tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> (tagId == -1) ? isHiveTag(tag) : (getTagId(tag) == tagId))
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range))
                .orElse(null);
    }

    public AprilTagDetection getBestDetection() {
        return getBestDetectionForTag(-1);
    }

    public TileCoordinate calculateRobotPosition(AprilTagDetection detection) {
        double[] pose = calculateRobotPoseWithoutUpdating(detection);
        if (pose == null) return null;

        TileCoordinate robotPos = new TileCoordinate(pose[0], pose[1]);
        if (driveSubsystem != null) {
            driveSubsystem.setPosition(robotPos);
            driveSubsystem.setHeading(pose[2]);
        }
        return robotPos;
    }

    public TileCoordinate getCurrentTilePosition() {
        AprilTagDetection best = getBestDetection();
        return (best == null) ? null : calculateRobotPosition(best);
    }

    public void updateBIOBUZZLocalizationTelemetry() {
        if (telemetry == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Vision FPS", visionPortal.getFps());
        telemetry.addData("Detections", detections.size());
        telemetry.addData("Hive State", currentHiveState.name());

        AprilTagDetection best = getBestDetection();
        if (best != null) {
            if (best instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) best;
                telemetry.addData("Best Target", "Cluster '%s' Range %.1f in (%d%% found)",
                        cluster.metadata != null ? cluster.metadata.shortName : "Unknown",
                        cluster.ftcPose.range, cluster.percentClusterFound);
            } else if (best instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) best;
                telemetry.addData("Best Target", "Single Tag ID %d Range %.1f in", single.id, single.ftcPose.range);
            }
        }
    }

    public void initializeCameraControls() {
        if (!cameraControlsSet && visionPortal != null) {
            if (setCameraControls(cameraExposureMs, cameraGain)) {
                cameraControlsSet = true;
            }
        }
    }

    public boolean setCameraControls(int exposureMs, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return false;
        try {
            ExposureControl exp = visionPortal.getCameraControl(ExposureControl.class);
            if (exp.getMode() != ExposureControl.Mode.Manual) {
                exp.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exp.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
            GainControl g = visionPortal.getCameraControl(GainControl.class);
            g.setGain(Range.clip(gain, g.getMinGain(), g.getMaxGain()));
            return true;
        } catch (Exception e) { return false; }
    }

    public List<AprilTagDetection> getRawDetections() { return aprilTag.getDetections(); }

    public void closeVision() { if (visionPortal != null) visionPortal.close(); }

    public double[] calculateAlignmentCorrections(AprilTagDetection tag, double desiredDist,
                                                  Double targetHeadingRad, double currentHeadingRad,
                                                  double dbX, double dbY, double dbAngleDeg,
                                                  double kPS, double kPF, double kPR, double kDS, double maxP) {
        if (tag == null || tag.ftcPose == null) {
            lastErrorX = 0;
            lastTime = 0;
            return new double[]{0, 0, 0, 0.0};
        }

        double currentTime = alignmentTimer.seconds();
        double deltaTime = (lastTime == 0) ? 0.02 : (currentTime - lastTime);

        double fErr = tag.ftcPose.y - desiredDist;
        double fP = (Math.abs(fErr) > dbY) ? fErr * kPF : 0;

        double errorX = tag.ftcPose.x;
        double dTermX = (deltaTime > 0) ? ((errorX - lastErrorX) / deltaTime) * kDS : 0;
        double sP = (Math.abs(errorX) > dbX) ? (errorX * kPS + dTermX) : 0;

        double tP = 0;
        double angleErrDeg;

        if (targetHeadingRad != null) {
            double headingErrRad = AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad);
            angleErrDeg = Math.toDegrees(headingErrRad);
            tP = headingErrRad * kPR * 57.29;
        } else {
            angleErrDeg = tag.ftcPose.bearing;
            tP = (Math.abs(angleErrDeg) > dbAngleDeg) ? angleErrDeg * kPR : 0;
        }

        lastErrorX = errorX;
        lastTime = currentTime;

        boolean aligned = Math.abs(fErr) <= dbY
                && Math.abs(errorX) <= dbX
                && Math.abs(angleErrDeg) <= dbAngleDeg;

        return new double[]{
                Range.clip(sP, -maxP, maxP),
                Range.clip(fP, -maxP, maxP),
                Range.clip(tP, -maxP, maxP),
                aligned ? 1.0 : 0.0
        };
    }

    public double[] calculateAlignmentCorrections(AprilTagDetection tag, double desiredDist, double desiredAngle,
                                                  double dbX, double dbY, double dbAngle,
                                                  double kPS, double kPF, double kPR, double maxP) {
        return calculateAlignmentCorrections(tag, desiredDist, null, 0, dbX, dbY, dbAngle, kPS, kPF, kPR, 0.03, maxP);
    }
}