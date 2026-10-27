package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import android.util.Size;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.vision.BioBuzzFieldMappingPipeline;
import org.firstinspires.ftc.teamcode.vision.OpenCvPipelineAdapter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Single VisionPortal owning the webcam. Hosts both AprilTagProcessor
 * (wrapped in logic via AprilTagNavigator) and the BioBuzz pollen pipeline.
 * Only one is active at a time via setProcessorEnabled — camera is opened once.
 */
public class VisionSubsystem {

    public enum Mode { APRIL_TAG, BIO_BUZZ }

    private static final double CAMERA_HEIGHT = 9.375;
    private static final double CAMERA_Y_OFFSET = 9.0;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final AprilTagNavigator aprilTagNavigator;
    private final BioBuzzFieldMappingPipeline bioBuzzPipeline;
    private final OpenCvPipelineAdapter bioBuzzAdapter;
    private Mode currentMode;

    public VisionSubsystem(Robot robot, DriveSubsystem driveSubsystem) {
        this(robot, driveSubsystem, Mode.BIO_BUZZ);
    }

    public VisionSubsystem(Robot robot, DriveSubsystem driveSubsystem, Mode startMode) {
        HardwareMap hwMap = robot.hardwareMap;

        Position cameraPosition = new Position(DistanceUnit.INCH, 0, CAMERA_Y_OFFSET, CAMERA_HEIGHT, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(270.376, 270.376, 153.340, 120.421) // calibrated for 320x240
                .build();
        aprilTag.setDecimation(2.0f);

        bioBuzzPipeline = new BioBuzzFieldMappingPipeline(robot.telemetry);
        bioBuzzAdapter = new OpenCvPipelineAdapter(bioBuzzPipeline);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240)) // matches calibrated intrinsics above
                .addProcessors(aprilTag, bioBuzzAdapter)
                .build();

        // FtcDashboard.getInstance().startCameraStream(visionPortal, 15);

        aprilTagNavigator = new AprilTagNavigator(driveSubsystem, aprilTag, visionPortal, robot.telemetry);

        setMode(startMode);
    }

    public void setMode(Mode mode) {
        if (mode == currentMode) return;
        boolean wantAprilTag = (mode == Mode.APRIL_TAG);
        visionPortal.setProcessorEnabled(aprilTag, wantAprilTag);
        visionPortal.setProcessorEnabled(bioBuzzAdapter, !wantAprilTag);
        currentMode = mode;
    }

    public Mode getMode() { return currentMode; }

    /** Full localization/alignment logic — getPedroPose, updateGlobalPoseEstimate, etc. */
    public AprilTagNavigator getAprilTagNavigator() { return aprilTagNavigator; }

    public BioBuzzFieldMappingPipeline.PollenTarget getBestTarget() {
        return bioBuzzPipeline.getPrimaryTarget();
    }

    public double getFps() { return visionPortal.getFps(); }

    /** Closes the shared camera — call once, from Robot teardown, not from AprilTagNavigator. */
    public void close() { visionPortal.close(); }

    public Command periodic() {
        return infinite(() -> {
            // Background vision tasks if needed
        });
    }
}