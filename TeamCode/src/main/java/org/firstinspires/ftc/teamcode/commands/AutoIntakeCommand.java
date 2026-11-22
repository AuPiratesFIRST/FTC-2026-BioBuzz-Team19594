package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.CameraCoordinateTransformer;
import org.firstinspires.ftc.teamcode.vision.BioBuzzFieldMappingPipeline;

import java.util.Collections;
import java.util.Set;

public class AutoIntakeCommand implements Command {
    private final Drivetrain drive;
    private final VisionSubsystem vision;
    private boolean hasStartedPath = false;

    public AutoIntakeCommand(Drivetrain drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
    }

    @Override
    public void start() {
        hasStartedPath = false;
    }

    @Override
    public void execute() {
        // Guard: Don't build a new path if we are already following one
        if (drive.getFollower().isBusy()) return;

        BioBuzzFieldMappingPipeline.PollenTarget target = vision.getBestTarget();

        if (target != null) {
            Pose robotPose = drive.getFollower().pose();

            // Project the pixel to a field-space Pose
            Pose targetPose = CameraCoordinateTransformer.projectPixelToField(target.bottomCenter, robotPose);
            if (targetPose == null) return;

            // Create Bezier curve path using Pedro 3 Paths API
            Pose controlPoint = new Pose(
                    targetPose.x() - 8 * Math.cos(robotPose.heading()),
                    targetPose.y() - 8 * Math.sin(robotPose.heading())
            );
            Path path = Paths.curve(robotPose, controlPoint, targetPose)
                    .constant(robotPose.heading());

            drive.getFollower().follow(path);
            hasStartedPath = true;
        }
    }

    @Override
    public boolean done() {
        // Command finishes when we've triggered a path and the robot stops being "busy"
        return hasStartedPath && !drive.getFollower().isBusy();
    }

    @Override
    public void end(EndCondition endCondition) {
        // If we want to stop or reset something when the path finishes
    }

    @Override
    public Set<Object> requirements() {
        return Collections.singleton(drive);
    }

    @Override
    public int priority() { return 0; }

    @Override
    public InterruptedBehavior interruptedBehavior() { return InterruptedBehavior.END; }

    @Override
    public ConflictBehavior conflictBehavior() { return ConflictBehavior.OVERRIDE; }

    @Override
    public BlockedBehavior blockedBehavior() { return BlockedBehavior.CANCEL; }
}