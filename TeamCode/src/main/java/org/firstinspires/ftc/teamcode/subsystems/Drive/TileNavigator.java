package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Handles high-level tile-based navigation logic for DECODE field.
 * 
 * This class translates field coordinates and tile positions into motor powers.
 * It uses the DriveSubsystem for low-level hardware control (motors, IMU).
 * 
 * Responsibilities:
 * - Position tracking (current tile coordinate)
 * - Coordinate transformations (field-to-robot conversions)
 * - Navigation algorithms (move to tile, turn to tile, etc.)
 * - Trigonometric calculations for pathfinding
 * 
 * Single Responsibility: Navigation logic only, no hardware control.
 */
public class TileNavigator {
    private final DriveSubsystem drive;
    private final Telemetry telemetry;
    
    // PID for position hold/navigation
    private double kpTranslational = 0.08;
    private double kdTranslational = 0.02;
    private double kpRotation = 1.8;
    
    // Constraints
    private double maxTranslationalPower = 0.7;
    private double maxRotationalPower = 0.5;
    
    private double lastDistError = 0;
    private long lastTime = 0;

    // Configuration
    private double translationalTolerance = 1.0; // inches
    private double headingTolerance = Math.toRadians(2.0);

    // Internal State: Current robot position in field coordinates
    private TileCoordinate currentPosition;
    private TileCoordinate targetPosition = null;
    private double targetHeading = 0;
    private boolean isNavigating = false;

    public TileNavigator(DriveSubsystem drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.currentPosition = new TileCoordinate(0, 0); // Start at origin (V1/A1)
    }

    // ==================== POSITION TRACKING ====================

    /**
     * Set the robot's current position in tile coordinates.
     * Call this after AprilTag localization to update "ground truth".
     *
     * @param position Current tile coordinate position
     */
    public void setPosition(TileCoordinate position) {
        this.currentPosition = position;
    }

    /**
     * Get the robot's current tile position.
     *
     * @return Current tile coordinate
     */
    public TileCoordinate getCurrentPosition() {
        return currentPosition;
    }

    // ==================== NAVIGATION METHODS ====================

    /**
     * Set a target position and heading for the robot to move to.
     * The robot will use PID control to reach and hold this pose.
     */
    public void setTargetPose(TileCoordinate target, double headingRad) {
        this.targetPosition = target;
        this.targetHeading = AngleUnit.normalizeRadians(headingRad);
        this.isNavigating = true;
        this.lastDistError = 0;
        this.lastTime = System.nanoTime();
    }

    /**
     * Helper to set target by tile coordinates.
     */
    public void setTargetTile(char col, int row, double headingDeg) {
        setTargetPose(new TileCoordinate(col, row, TileCoordinate.TILE_SIZE/2, TileCoordinate.TILE_SIZE/2), 
                     Math.toRadians(headingDeg));
    }

    /**
     * Stop the current navigation task.
     */
    public void stopNavigation() {
        this.isNavigating = false;
        this.targetPosition = null;
        drive.stop();
    }

    /**
     * Updates the navigation control loop. Call this repeatedly in your OpMode.
     * Uses PD control for translation and P control for rotation.
     */
    public void update() {
        // First, sync current position from drive subsystem odometry
        this.currentPosition = drive.getCurrentPosition();
        
        if (!isNavigating || targetPosition == null) {
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double distance = currentPosition.distanceTo(targetPosition);
        double angleToTarget = currentPosition.angleTo(targetPosition);
        double currentHeading = drive.getHeading();

        // 1. Translational Error (Forward/Strafe)
        double derivative = dt > 0 ? (distance - lastDistError) / dt : 0;
        lastDistError = distance;

        double translationalPower = (distance * kpTranslational) + (derivative * kdTranslational);
        translationalPower = Range.clip(translationalPower, -maxTranslationalPower, maxTranslationalPower);

        // Convert field-relative angle to robot-relative components
        double relativeAngle = AngleUnit.normalizeRadians(angleToTarget - currentHeading);
        double forward = Math.cos(relativeAngle) * translationalPower;
        double strafe = Math.sin(relativeAngle) * translationalPower;

        // 2. Rotational Error (Heading)
        double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading);
        double turn = headingError * kpRotation;
        turn = Range.clip(turn, -maxRotationalPower, maxRotationalPower);

        // 3. Arrival Logic
        // If we are very close to target, stop but keep holding heading
        if (distance < translationalTolerance) {
            forward = 0;
            strafe = 0;
        }
        
        // If both distance and heading are within tolerance, we are done
        if (distance < translationalTolerance && Math.abs(headingError) < headingTolerance) {
            isNavigating = false;
            drive.stop();
        } else {
            drive.drive(forward, strafe, turn);
        }

        if (telemetry != null) {
            telemetry.addData("Nav Target", targetPosition.getTilePosition());
            telemetry.addData("Dist Err", "%.2f", distance);
            telemetry.addData("Head Err", "%.1f°", Math.toDegrees(headingError));
        }
    }

    /**
     * Returns true if the robot has arrived at the target position and heading.
     */
    public boolean isAtTarget() {
        if (!isNavigating) return true;
        
        double distance = currentPosition.distanceTo(targetPosition);
        double headingError = AngleUnit.normalizeRadians(targetHeading - drive.getHeading());
        
        return distance < translationalTolerance && Math.abs(headingError) < headingTolerance;
    }

    /**
     * Move to a specific tile coordinate position using legacy logic.
     * @deprecated Use setTargetPose() and update() for smoother PID movement.
     */
    @Deprecated
    public void moveToPosition(TileCoordinate target, double power) {
        setTargetPose(target, drive.getHeading());
        kpTranslational = power * 0.1; // adjust gain to match requested power
    }

    /**
     * Move to a specific tile with offset.
     *
     * @param column  Tile column (A-F)
     * @param row     Tile row (1-6)
     * @param offsetX Offset within tile (0-24 inches)
     * @param offsetY Offset within tile (0-24 inches)
     * @param power   Motor power (0-1)
     */
    public void moveToTile(char column, int row, double offsetX, double offsetY, double power) {
        TileCoordinate target = new TileCoordinate(column, row, offsetX, offsetY);
        moveToPosition(target, power);
    }

    /**
     * Move to center of a specific tile.
     *
     * @param column Tile column (A-F)
     * @param row    Tile row (1-6)
     * @param power  Motor power (0-1)
     */
    public void moveToTileCenter(char column, int row, double power) {
        moveToTile(column, row, TileCoordinate.TILE_SIZE / 2, TileCoordinate.TILE_SIZE / 2, power);
    }

    /**
     * Move to a tab-line intersection.
     *
     * @param column Tab column (V-Z)
     * @param row    Tab row (1-5)
     * @param power  Motor power (0-1)
     */
    public void moveToTabIntersection(char column, int row, double power) {
        TileCoordinate target = new TileCoordinate(column, row);
        moveToPosition(target, power);
    }

    /**
     * Move relative to current position by tile units.
     *
     * Performs coordinate transformation to move relative to robot's current heading.
     * Converts relative movement (forward/right) to absolute field coordinates:
     *
     * newX = currentX + forward×cos(heading) - strafe×sin(heading)
     * newY = currentY + forward×sin(heading) + strafe×cos(heading)
     *
     * This rotation matrix accounts for robot orientation, so "forward" means
     * in the direction the robot is facing, not always field-north.
     *
     * @param tilesForward Number of tiles forward (negative for backward)
     * @param tilesRight   Number of tiles right (negative for left)
     * @param power        Motor power (0-1)
     */
    public void moveRelativeTiles(double tilesForward, double tilesRight, double power) {
        // Convert tiles to inches
        double forward = tilesForward * TileCoordinate.TILE_SIZE;
        double strafe = tilesRight * TileCoordinate.TILE_SIZE;

        // Coordinate transformation: rotate relative movement by current heading
        // Uses rotation matrix to convert robot-relative to field-absolute coordinates
        double currentHeading = drive.getHeading();
        double newX = currentPosition.getX() + forward * Math.cos(currentHeading) - strafe * Math.sin(currentHeading);
        double newY = currentPosition.getY() + forward * Math.sin(currentHeading) + strafe * Math.cos(currentHeading);

        TileCoordinate target = new TileCoordinate(newX, newY);
        moveToPosition(target, power);
    }

    /**
     * Turn to face a specific tile.
     *
     * Calculates shortest rotation angle to face target:
     * - turnAngle = targetAngle - currentHeading
     * - Normalized to [-π, π] range to ensure shortest rotation path
     * - Uses signum() to determine turn direction (positive = CCW, negative = CW)
     *
     * @param target Target tile coordinate
     * @param power  Turn power (0-1)
     */
    public void turnToTile(TileCoordinate target, double power) {
        double angle = currentPosition.angleTo(target);
        double currentHeading = drive.getHeading();
        double turnAngle = angle - currentHeading;

        // Normalize turn angle to [-π, π] range for shortest rotation
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

        // Use signum to get direction: +1 for CCW, -1 for CW
        double turn = Math.signum(turnAngle) * power;
        drive.drive(0, 0, turn);

        if (telemetry != null) {
            telemetry.addData("Turn to Tile", "Facing %s", target.getTilePosition());
            telemetry.addData("Turn Angle", "%.1f degrees", Math.toDegrees(turnAngle));
            telemetry.update();
        }
    }

    // ==================== QUERY METHODS ====================

    /**
     * Get distance to a specific tile.
     *
     * @param target Target tile coordinate
     * @return Distance in inches
     */
    public double getDistanceToTile(TileCoordinate target) {
        return currentPosition.distanceTo(target);
    }

    /**
     * Get angle to a specific tile.
     *
     * @param target Target tile coordinate
     * @return Angle in radians
     */
    public double getAngleToTile(TileCoordinate target) {
        return currentPosition.angleTo(target);
    }

    /**
     * Check if robot is at a specific tile (within tolerance).
     *
     * @param target    Target tile coordinate
     * @param tolerance Tolerance in inches
     * @return True if within tolerance
     */
    public boolean isAtTile(TileCoordinate target, double tolerance) {
        return getDistanceToTile(target) <= tolerance;
    }

    // ==================== TELEMETRY ====================

    /**
     * Update telemetry with tile-based navigation information.
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Tile", currentPosition.getTilePosition());
            telemetry.addData("Current Tab", currentPosition.getTabPosition());
            telemetry.addData("Position (inches)", "X: %.1f, Y: %.1f", currentPosition.getX(), currentPosition.getY());
            telemetry.addData("Heading", "%.1f degrees", drive.getHeadingDegrees()); // Use getHeadingDegrees() for telemetry

            double[] offset = currentPosition.getTileOffset();
            telemetry.addData("Tile Offset", "X: %.1f, Y: %.1f", offset[0], offset[1]);
        }
    }
}
