package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.vision.AprilTagNavigator;

/**
 * Centralized field- and game-specific constants for the 2026-2027 BIOBUZZ season.
 */
public final class FieldConstants {

    private FieldConstants() {}

    // Field Dimensions (Per Section 9.1 & 9.4)
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_HEIGHT_INCHES = 144.0;
    public static final double TILE_SIZE_INCHES = 24.0;

    // AprilTag Specifications (Section 9.9)
    // 3.25 inch square targets from the 36h11 family
    public static final double APRILTAG_SIZE_INCHES = 3.25;
    public static final double APRILTAG_SIZE_CM = 8.255;

    /**
     * Alliance tag positions: {tagId, x_inches, y_inches, heading_degrees}
     * Note: IDs 30-45 represent the HIVE and Perimeter clusters for BIOBUZZ.
     * Reference official field CAD and Section 9.9 of the Competition Manual for zero-point alignment.
     * These coordinates are translated to the (0, 144) corner-origin system.
     */
    public static final double[][] APRILTAG_POSITIONS = new double[][]{
        // Red Backstage Cluster (Back wall, Left side)
        {30, 36.0, 132.0, 180.0}, {31, 48.0, 132.0, 180.0}, {32, 60.0, 132.0, 180.0}, {33, 72.0, 132.0, 180.0},
        // Red Audience Cluster (Front wall, Left side)
        {34, 36.0, 12.0, 0.0}, {35, 48.0, 12.0, 0.0}, {36, 60.0, 12.0, 0.0}, {37, 72.0, 12.0, 0.0},
        // Blue Audience Cluster (Front wall, Right side)
        {38, 72.0, 12.0, 0.0}, {39, 84.0, 12.0, 0.0}, {40, 96.0, 12.0, 0.0}, {41, 108.0, 12.0, 0.0},
        // Blue Backstage Cluster (Back wall, Right side)
        {42, 72.0, 132.0, 180.0}, {43, 84.0, 132.0, 180.0}, {44, 96.0, 132.0, 180.0}, {45, 108.0, 132.0, 180.0},
        // HIVE-Mounted Tags (Nominal positions at rest)
        // Red CELL (Audience side): {tagId, x, y, heading, z}
        {4, 60.0, 60.0, 0.0, 36.95}, {5, 66.0, 60.0, 0.0, 36.95},
        {6, 78.0, 60.0, 0.0, 36.95}, {7, 84.0, 60.0, 0.0, 36.95}
    };

    // Game Structure Offsets (Section 9.6 HIVE Structure & Section 9.7 FLOWER)
    public static final double HIVE_CENTER_X = 72.0;
    public static final double HIVE_CENTER_Y = 72.0;
    public static final double HIVE_PIVOT_HEIGHT_INCHES = 43.95;
    
    // FLOWER structures are at ~21.5" height
    public static final double FLOWER_TARGET_HEIGHT_INCHES = 21.5; 

    // ==================== BIOBUZZ STARTING POSITIONS ====================
    // Verify exact tile coordinates (Section 9.3 & 9.4) and starting heading (Section 10.3)
    
    // Red Alliance Start Position (Typical D1 center)
    public static final double START_X_RED = 84.0;      
    public static final double START_Y_RED = 12.0;      
    public static final double START_HEADING_DEGREES_RED = 90.0;

    // Blue Alliance Start Position (Typical C1 center)
    public static final double START_X_BLUE = 60.0;     
    public static final double START_Y_BLUE = 12.0;     
    public static final double START_HEADING_DEGREES_BLUE = 90.0;

    /**
     * Returns the default field coordinates {x, y, z, heading} for a HIVE AprilTag cluster.
     * Note: Z-height reflects the bottom face of the CELL in its resting position.
     */
    // Lower CELL Resting Tag Centerline Height
    public static final double HIVE_TAG_Z_LOWER = 28.25;

    // Upper CELL Tag Centerline Height
    public static final double HIVE_TAG_Z_UPPER = 56.25;

    public static double[] getClusterPosition(String clusterName, AprilTagNavigator.HiveState hiveState) {
        if (clusterName == null) return null;

        // Select correct baseline Z depending on whether the structure is Tilted Up or Down
        double tagZ = (hiveState == AprilTagNavigator.HiveState.RED_TILTED_UP || hiveState == AprilTagNavigator.HiveState.BLUE_TILTED_UP)
                ? HIVE_TAG_Z_UPPER
                : HIVE_TAG_Z_LOWER;

        switch (clusterName) {
            case "RED SCORING":   return new double[]{54.0, 132.0, tagZ, 180.0};
            case "RED AUDIENCE":  return new double[]{54.0,  12.0, tagZ,   0.0};
            case "BLUE AUDIENCE": return new double[]{90.0,  12.0, tagZ,   0.0};
            case "BLUE SCORING":  return new double[]{90.0, 132.0, tagZ, 180.0};
            default: return null;
        }
    }
}
