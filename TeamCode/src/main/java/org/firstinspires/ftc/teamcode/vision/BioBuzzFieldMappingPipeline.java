package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Merged BIOBUZZ pollen field-mapping pipeline (FTC 2026-2027).
 *
 * Combines:
 *  - Perspective-aware area filtering + speckle suppression + region exclusion
 *    (from the "cinematic footage" version)
 *  - Convexity filtering + cross-frame hysteresis/locking
 *    (from the original trig-based version)
 *  - Standard OpenCV4 calls throughout (no OpenCVCompat shim) — Imgproc/Core
 *    methods here are the same ones EasyOpenCV ships with on-robot.
 *
 * Field-relative position is intentionally NOT computed here. Feed
 * PollenTarget.bottomCenter directly into CameraCoordinateTransformer
 * .projectPixelToField(pixel, robotPose) downstream — doing it there once
 * (using real robot heading from odometry/AprilTag) is both more correct
 * and cheaper than computing camera-relative trig here and discarding it.
 */
public class BioBuzzFieldMappingPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    public static class PollenTarget {
        public Point bottomCenter;
        public Rect boundingBox;
        public double priorityScore;
        public double circularity;
        public double convexity;
        public int framesSeen = 0;
        public boolean locked = false;

        public PollenTarget(Point bottomCenter, Rect boundingBox, double priorityScore,
                             double circularity, double convexity) {
            this.bottomCenter = bottomCenter;
            this.boundingBox = boundingBox;
            this.priorityScore = priorityScore;
            this.circularity = circularity;
            this.convexity = convexity;
        }
    }

    // --- Color threshold (HSV) ---
    public int yellowLowerH = 15, yellowLowerS = 120, yellowLowerV = 100;
    public int yellowUpperH = 35, yellowUpperS = 255, yellowUpperV = 255;

    // --- Perspective-based geometric tuning ---
    public double minAreaHorizon = 120,   maxAreaHorizon = 2200;
    public double minAreaIntake  = 2500,  maxAreaIntake  = 35000;
    public double minAspectRatio = 0.55,  maxAspectRatio = 1.45;

    // --- Shape quality thresholds ---
    public double minCircularity = 0.60;
    public double minConvexity   = 0.70;

    // --- Morphology ---
    public int holeFillingKernelSize = 13;
    public int erosionKernelSize     = 5;
    public boolean enableSpeckleSuppression = true;
    public int speckleOpenKernelSize = 3;

    // --- Region-of-interest bands ---
    public int topExclusionY    = 120;
    public int intakeExclusionY = 350;
    public double centeringWeight = 0.75;

    // --- Hysteresis tuning ---
    public double matchDistancePx = 30.0;
    public int    lockAfterFrames = 5;
    public int    confirmAfterFrames = 3;

    private final List<PollenTarget> activePollenList = new ArrayList<>();
    private final List<PollenTarget> persistentTargets = new ArrayList<>();
    private final List<PollenTarget> lastFrameDetections = new ArrayList<>();
    private PollenTarget primaryTarget = null;
    private int rejectedByCircularity = 0;
    private int rejectedByConvexity = 0;

    // Reusable Mat objects and contour list to eliminate GC / native allocation per frame
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat rgb = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();

    public BioBuzzFieldMappingPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        long frameStartNanos = System.nanoTime();

        activePollenList.clear();
        primaryTarget = null;
        rejectedByCircularity = 0;
        rejectedByConvexity = 0;

        // Release contours from previous frame
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        contours.clear();

        try {
            // EasyOpenCV commonly delivers RGBA; drop alpha before HSV conversion.
            if (input.channels() == 4) {
                Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
            } else {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            }

            Core.inRange(hsv,
                    new Scalar(yellowLowerH, yellowLowerS, yellowLowerV),
                    new Scalar(yellowUpperH, yellowUpperS, yellowUpperV),
                    mask);

            // 1. SPECKLE SUPPRESSION (small kernel, before hole-fusing)
            if (enableSpeckleSuppression && speckleOpenKernelSize > 0) {
                Mat openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                        new Size(speckleOpenKernelSize, speckleOpenKernelSize));
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
                openKernel.release();
            }

            // 2. HOLE-FUSING (large kernel) — must come before erosion, or
            // wiffle-ball perforations fragment into rings before they can be filled.
            Mat closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                    new Size(holeFillingKernelSize, holeFillingKernelSize));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);
            closeKernel.release();

            // 3. CLUSTER SEPARATION
            Mat erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                    new Size(erosionKernelSize, erosionKernelSize));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_ERODE, erodeKernel);
            erodeKernel.release();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int imageCenterX = input.width() / 2;
            double exclusionSpan = intakeExclusionY - topExclusionY;
            List<PollenTarget> foundThisFrame = new ArrayList<>();

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.height <= 0 || rect.width <= 0) continue;

                double aspect = (double) rect.width / rect.height;
                double centerX = rect.x + (rect.width / 2.0);
                double currentBottomY = rect.y + rect.height;

                // Region gating
                if (rect.y < topExclusionY || currentBottomY >= intakeExclusionY) continue;
                if (aspect < minAspectRatio || aspect > maxAspectRatio) continue;

                // Perspective-interpolated area thresholds
                double normY = (exclusionSpan > 0)
                        ? (currentBottomY - topExclusionY) / exclusionSpan : 0.0;
                normY = Math.max(0.0, Math.min(1.0, normY));
                double dynamicMinArea = minAreaHorizon + (normY * (minAreaIntake - minAreaHorizon));
                double dynamicMaxArea = maxAreaHorizon + (normY * (maxAreaIntake - maxAreaHorizon));
                if (area < dynamicMinArea || area > dynamicMaxArea) continue;

                // Convexity (rejects concave reflection artifacts)
                MatOfInt hullIndices = new MatOfInt();
                Imgproc.convexHull(contour, hullIndices);
                double hullArea = calculateHullArea(contour, hullIndices);
                hullIndices.release();
                double convexity = (hullArea > 0) ? area / hullArea : 0;
                if (convexity < minConvexity) {
                    rejectedByConvexity++;
                    continue;
                }

                // Circularity (rejects distorted/flattened blobs)
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double circularity;
                try {
                    double perimeter = Imgproc.arcLength(contour2f, true);
                    circularity = (perimeter > 0) ? (4 * Math.PI * area) / (perimeter * perimeter) : 0;
                } finally {
                    contour2f.release();
                }
                if (circularity < minCircularity) {
                    rejectedByCircularity++;
                    Imgproc.rectangle(input, rect, new Scalar(120, 120, 120), 1);
                    continue;
                }

                double distanceScore = currentBottomY;
                double centerDeviation = Math.abs(centerX - imageCenterX);
                double centeringPenalty = Math.pow(centerDeviation, 1.2) * centeringWeight;
                double finalScore = distanceScore - centeringPenalty;

                Point bottomCenterPoint = new Point(centerX, currentBottomY);
                foundThisFrame.add(new PollenTarget(bottomCenterPoint, rect, finalScore, circularity, convexity));
            }

            // --- HYSTERESIS: match against persistent targets ---
            for (PollenTarget found : foundThisFrame) {
                boolean matched = false;
                for (PollenTarget p : persistentTargets) {
                    double dist = Math.hypot(found.bottomCenter.x - p.bottomCenter.x,
                                              found.bottomCenter.y - p.bottomCenter.y);
                    if (dist < matchDistancePx) {
                        p.bottomCenter = found.bottomCenter;
                        p.boundingBox = found.boundingBox;
                        p.priorityScore = found.priorityScore;
                        p.circularity = found.circularity;
                        p.convexity = found.convexity;
                        p.framesSeen++;
                        if (p.framesSeen > lockAfterFrames) p.locked = true;
                        matched = true;
                        break;
                    }
                }
                if (!matched) {
                    found.framesSeen = 1;
                    persistentTargets.add(found);
                }
            }

            persistentTargets.removeIf(p -> {
                boolean stillThere = false;
                for (PollenTarget f : foundThisFrame) {
                    if (Math.hypot(f.bottomCenter.x - p.bottomCenter.x,
                                    f.bottomCenter.y - p.bottomCenter.y) < matchDistancePx) {
                        stillThere = true;
                        break;
                    }
                }
                if (!stillThere) p.framesSeen--;
                return p.framesSeen <= 0;
            });

            for (PollenTarget p : persistentTargets) {
                if (p.framesSeen >= confirmAfterFrames) activePollenList.add(p);
            }

            activePollenList.sort(Comparator.comparingDouble((PollenTarget p) -> p.priorityScore).reversed());
            if (!activePollenList.isEmpty()) primaryTarget = activePollenList.get(0);

            lastFrameDetections.clear();
            lastFrameDetections.addAll(foundThisFrame);

            // --- VISUALIZATION: rank-coded, green/cyan/blue ---
            for (int i = 0; i < activePollenList.size(); i++) {
                PollenTarget pollen = activePollenList.get(i);
                Scalar color;
                int thickness;
                if (i == 0)      { color = new Scalar(0, 255, 0);   thickness = 3; }
                else if (i == 1) { color = new Scalar(0, 255, 255); thickness = 2; }
                else             { color = new Scalar(255, 100, 0); thickness = 1; }
                Imgproc.rectangle(input, pollen.boundingBox, color, thickness);
            }

            if (primaryTarget != null) {
                Imgproc.circle(input, primaryTarget.bottomCenter, 5, new Scalar(0, 0, 255), -1);
                String label = String.format("LOCK - SCORE: %.1f%s", primaryTarget.priorityScore,
                        primaryTarget.locked ? " [LOCKED]" : "");
                Imgproc.putText(input, label,
                        new Point(primaryTarget.boundingBox.x, primaryTarget.boundingBox.y - 8),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, new Scalar(0, 255, 0), 2);
            }

            Imgproc.line(input, new Point(0, topExclusionY), new Point(input.width(), topExclusionY),
                    new Scalar(0, 0, 255), 2);
            Imgproc.line(input, new Point(0, intakeExclusionY), new Point(input.width(), intakeExclusionY),
                    new Scalar(255, 100, 0), 2);
            Imgproc.line(input, new Point(imageCenterX, 0), new Point(imageCenterX, input.height()),
                    new Scalar(220, 220, 220), 1);

            double frameMs = (System.nanoTime() - frameStartNanos) / 1_000_000.0;

            telemetry.addLine("=== BIOBUZZ POLLEN TRACKER ===");
            telemetry.addData("Confirmed Targets", activePollenList.size());
            telemetry.addData("Persistent (unconfirmed incl.)", persistentTargets.size());
            telemetry.addData("Rejected (circularity)", rejectedByCircularity);
            telemetry.addData("Rejected (convexity)", rejectedByConvexity);
            telemetry.addData("Lock Status", primaryTarget != null
                    ? (primaryTarget.locked ? "LOCKED" : "TRACKING") : "SCANNING FIELD");
            telemetry.addData("Frame Time (ms)", String.format("%.2f", frameMs));
            telemetry.update();

            return input;
        } finally {
            for (MatOfPoint contour : contours) {
                contour.release();
            }
            contours.clear();
        }
    }

    /**
     * Releases native OpenCV memory on pipeline teardown.
     */
    public void release() {
        if (hsv != null) hsv.release();
        if (mask != null) mask.release();
        if (hierarchy != null) hierarchy.release();
        if (rgb != null) rgb.release();
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        contours.clear();
    }

    private double calculateHullArea(MatOfPoint contour, MatOfInt hullIndices) {
        Point[] pts = contour.toArray();
        int[] idx = hullIndices.toArray();
        List<Point> hullPts = new ArrayList<>();
        for (int i : idx) hullPts.add(pts[i]);
        MatOfPoint hull = new MatOfPoint();
        hull.fromList(hullPts);
        try { return Imgproc.contourArea(hull); }
        finally { hull.release(); }
    }

    public List<PollenTarget> getActivePollenList()      { return new ArrayList<>(activePollenList); }
    public List<PollenTarget> getLastFrameDetections()    { return new ArrayList<>(lastFrameDetections); }
    public int getPersistentTargetCount()                 { return persistentTargets.size(); }
    public PollenTarget getPrimaryTarget()                { return primaryTarget; }

    public void clearTargets() {
        activePollenList.clear();
        persistentTargets.clear();
        lastFrameDetections.clear();
        primaryTarget = null;
    }
}
