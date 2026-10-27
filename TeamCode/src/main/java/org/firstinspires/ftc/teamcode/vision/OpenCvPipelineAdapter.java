package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Bridges an EasyOpenCV OpenCvPipeline into the VisionPortal's VisionProcessor
 * contract, so it can be registered alongside AprilTagProcessor on one portal
 * and toggled with VisionPortal.setProcessorEnabled(...).
 */
public class OpenCvPipelineAdapter implements VisionProcessor {
    private final OpenCvPipeline pipeline;

    public OpenCvPipelineAdapter(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // BioBuzzFieldMappingPipeline has no init hook; nothing required.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // BioBuzzFieldMappingPipeline draws directly onto the Mat it returns,
        // so no separate onDrawFrame work is needed.
        return pipeline.processFrame(frame);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No-op: drawing already happened inside processFrame().
    }
}