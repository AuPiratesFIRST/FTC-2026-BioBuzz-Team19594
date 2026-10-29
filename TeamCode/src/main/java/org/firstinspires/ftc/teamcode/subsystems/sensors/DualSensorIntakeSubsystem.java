package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

/**
 * High-Speed Dual-Sensor Intake & Dynamic Over-Center Hood Linkage Subsystem.
 *
 * Tracks up to 4 game elements in a FIFO queue (LinkedList pattern) through a
 * 2-stage sensor array:
 * 1. Entry Sensor (ramp base): Detects ball type (YELLOW_BALL, RED_BALL, BLUE_BALL, UNKNOWN)
 * and queues it.
 * 2. Top Sensor (pre-shooter): Verifies staged ball before flywheel entry and
 * tracks launches.
 */
public class DualSensorIntakeSubsystem {

    // Maximum capacity for 4-ball pattern FIFO queue
    public static final int MAX_QUEUE_CAPACITY = 4;

    // Servo positions for Over-Center Linkage hard stops
    public static final double HOOD_SMALL_BALL_POS = 0.85; // Locked over-center (Small / Yellow Pollen)
    public static final double HOOD_LARGE_BALL_POS = 0.15; // Retracted position (Large / Alliance ball)

    // Distance threshold in inches to register presence
    public static final double DETECTION_THRESHOLD_INCHES = 3.5;

    public enum BallType {
        YELLOW_BALL,   // Small Pollen element (Yellow)
        RED_BALL,      // Red Alliance element
        BLUE_BALL,     // Blue Alliance element
        ALLIANCE_BALL, // Generic Alliance element fallback
        UNKNOWN        // Unclassified / Noise
    }

    private final ColorRangeSensor entrySensor;
    private final ColorRangeSensor topSensor;
    private final ServoImplEx hoodServo;

    private final LinkedList<BallType> ballQueue = new LinkedList<>();
    private final boolean isRedAlliance;

    // Diagnostic tracking
    private double lastEntryDist = 999.0;
    private double lastTopDist = 999.0;
    private boolean entryWasTriggered = false;
    private boolean topWasTriggered = false;

    // Multi-sample color accumulation
    private float sumR = 0f, sumG = 0f, sumB = 0f;
    private int sampleCount = 0;
    private BallType lastIdentifiedBall = BallType.UNKNOWN;
    private float lastGRRatio = 0f;
    private String lastRawColorStr = "R:0.00 G:0.00 B:0.00 | Hue:0° Sat:0.00 G/R:0.00";

    // Hood state tracking
    private double currentHoodTarget = HOOD_LARGE_BALL_POS;

    public DualSensorIntakeSubsystem(HardwareMap hardwareMap, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;

        // Hardware mapping
        entrySensor = hardwareMap.get(ColorRangeSensor.class, "entry_sensor");
        topSensor = hardwareMap.get(ColorRangeSensor.class, "top_sensor");
        hoodServo = hardwareMap.get(ServoImplEx.class, "hood_servo");

        // Enable built-in light for both sensors
        if (entrySensor instanceof SwitchableLight) {
            ((SwitchableLight) entrySensor).enableLight(true);
        }
        if (topSensor instanceof SwitchableLight) {
            ((SwitchableLight) topSensor).enableLight(true);
        }

        // Set gain for normalized color sensitivity
        entrySensor.setGain(4.0f);
        topSensor.setGain(4.0f);

        // Expand PWM range to 500-2500µsec for goBILDA 2000 Series 25-4 Super Speed servo
        hoodServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Default to retracted large ball position on init
        setHoodPosition(HOOD_LARGE_BALL_POS);
    }

    // Frame-skipping counter for I2C bus throttling
    private int sensorPollCounter = 0;
    private static final int POLL_INTERVAL_CYCLES = 3; // Poll sensor every 3rd loop cycle

    /**
     * Periodic update loop with rate-limited I2C sensor polling.
     */
    public void update() {
        // ==================== SENSOR 1: ENTRY CHECK (Intake Base) ====================
        double entryDist = lastEntryDist;

        // Throttled I2C polling to keep loop frequency blazing fast (~200+ Hz)
        sensorPollCounter++;
        if (sensorPollCounter >= POLL_INTERVAL_CYCLES) {
            sensorPollCounter = 0;

            try {
                entryDist = entrySensor.getDistance(DistanceUnit.INCH);
            } catch (Exception ignored) {}
            if (Double.isNaN(entryDist) || Double.isInfinite(entryDist)) {
                entryDist = 999.0;
            }
            lastEntryDist = entryDist;

            // Only sample colors when active polling occurs and object is close
            if (entryDist < DETECTION_THRESHOLD_INCHES) {
                NormalizedRGBA c = entrySensor.getNormalizedColors();
                if (c.alpha > 0.005f) {
                    float normR = c.red / c.alpha;
                    float normG = c.green / c.alpha;
                    float normB = c.blue / c.alpha;

                    sumR += normR;
                    sumG += normG;
                    sumB += normB;
                    sampleCount++;

                    float avgR = sumR / sampleCount;
                    float avgG = sumG / sampleCount;
                    float avgB = sumB / sampleCount;

                    lastGRRatio = avgG / Math.max(avgR, 0.001f);

                    float[] hsv = new float[3];
                    android.graphics.Color.colorToHSV(c.toColor(), hsv);

                    lastRawColorStr = String.format(Locale.US,
                            "R:%.2f G:%.2f B:%.2f | Hue:%.0f° Sat:%.2f G/R:%.2f",
                            normR, normG, normB, hsv[0], hsv[1], lastGRRatio);

                    lastIdentifiedBall = classify(avgR, avgG, avgB);
                }
            }
        }

        boolean entryTriggered = (entryDist < DETECTION_THRESHOLD_INCHES);

        if (entryTriggered && !entryWasTriggered) {
            // Rising edge: A new ball entered the ramp
            if (ballQueue.size() < MAX_QUEUE_CAPACITY) {
                if (lastIdentifiedBall != BallType.UNKNOWN) {
                    ballQueue.addLast(lastIdentifiedBall);
                    if (ballQueue.size() == 1) {
                        applyHoodForBallType(lastIdentifiedBall);
                    }
                }
            }
        } else if (entryTriggered && entryWasTriggered) {
            // Continuously refine classification while ball sits in entry zone
            if (!ballQueue.isEmpty() && lastIdentifiedBall != BallType.UNKNOWN) {
                ballQueue.set(ballQueue.size() - 1, lastIdentifiedBall);
                if (ballQueue.size() == 1) {
                    applyHoodForBallType(lastIdentifiedBall);
                }
            }
        } else if (!entryTriggered && entryWasTriggered) {
            // Falling edge: Ball passed entry sensor; reset accumulators
            sumR = 0f;
            sumG = 0f;
            sumB = 0f;
            sampleCount = 0;
        }
        entryWasTriggered = entryTriggered;


        // ==================== SENSOR 2: TOP TRANSFER CHECK (Pre-Shooter) ====================
        double topDist = 999.0;
        try {
            topDist = topSensor.getDistance(DistanceUnit.INCH);
        } catch (Exception ignored) {}
        if (Double.isNaN(topDist) || Double.isInfinite(topDist)) {
            topDist = 999.0;
        }
        lastTopDist = topDist;
        boolean topTriggered = (topDist < DETECTION_THRESHOLD_INCHES);

        if (topTriggered && !topWasTriggered) {
            // Rising edge: Ball reached staging chamber
            if (!ballQueue.isEmpty()) {
                BallType stagedBall = ballQueue.peekFirst();
                applyHoodForBallType(stagedBall);
            }
        } else if (!topTriggered && topWasTriggered) {
            // Falling edge: Ball fed into flywheel chamber & launched!
            onBallLaunched();
        }
        topWasTriggered = topTriggered;
    }

    /**
     * Classifies element using alpha-normalized RGB and Green-to-Red ratio.
     */
    public BallType classify(float r, float g, float b) {
        // 1. Blue ball check: Blue must dominate red and green
        if (b > r * 1.15f && b > g * 1.0f) {
            return BallType.BLUE_BALL;
        }

        // Calculate Green-to-Red ratio
        float gr = g / Math.max(r, 0.001f);

        // 2. Yellow Pollen check: Green and Red are balanced (Ratio ~1.0 to 1.1)
        if (gr >= 0.90f) {
            return BallType.YELLOW_BALL;
        }

        // 3. Red Ball check: Red is higher than Green (Ratio observed around 0.65 - 0.70)
        if (gr >= 0.50f && gr < 0.90f) {
            return BallType.RED_BALL;
        }

        // Fallback for deep red (Ratio < 0.50)
        if (gr < 0.50f) {
            return BallType.RED_BALL;
        }

        return BallType.UNKNOWN;
    }

    public BallType identifyBall(ColorRangeSensor sensor) {
        return lastIdentifiedBall;
    }

    public void onBallLaunched() {
        if (!ballQueue.isEmpty()) {
            BallType launched = ballQueue.pollFirst();

            if (!ballQueue.isEmpty()) {
                BallType nextBall = ballQueue.peekFirst();
                if (nextBall != launched) {
                    applyHoodForBallType(nextBall);
                }
            }
        }
    }

    private void applyHoodForBallType(BallType type) {
        if (type == BallType.YELLOW_BALL) {
            setHoodPosition(HOOD_SMALL_BALL_POS);
        } else {
            setHoodPosition(HOOD_LARGE_BALL_POS);
        }
    }

    private void setHoodPosition(double pos) {
        if (Math.abs(currentHoodTarget - pos) > 0.01) {
            currentHoodTarget = pos;
            hoodServo.setPosition(pos);
        }
    }

    // ==================== DIAGNOSTIC & PATTERN QUERY METHODS ====================

    public double getEntryDistance() {
        return lastEntryDist;
    }

    public double getTopDistance() {
        return lastTopDist;
    }

    public boolean isEntryTriggered() {
        return entryWasTriggered;
    }

    public String getEntryRawColorString() {
        return lastRawColorStr;
    }

    public BallType getDetectedEntryColor() {
        return lastIdentifiedBall;
    }

    public float getLastGRRatio() {
        return lastGRRatio;
    }

    public List<BallType> getQueuePattern() {
        return Collections.unmodifiableList(new ArrayList<>(ballQueue));
    }

    public boolean isAllPollen() {
        if (ballQueue.isEmpty()) return false;
        for (BallType type : ballQueue) {
            if (type != BallType.YELLOW_BALL) return false;
        }
        return true;
    }

    public boolean isAllianceBall(BallType type) {
        if (type == BallType.ALLIANCE_BALL) return true;
        if (isRedAlliance && type == BallType.RED_BALL) return true;
        return !isRedAlliance && type == BallType.BLUE_BALL;
    }

    public boolean isAllAlliance() {
        if (ballQueue.isEmpty()) return false;
        for (BallType type : ballQueue) {
            if (!isAllianceBall(type)) return false;
        }
        return true;
    }

    public BallType getStagedBall() {
        return ballQueue.peekFirst();
    }

    public boolean isBallStagedAtTop() {
        return topWasTriggered;
    }

    public boolean isOverCenterLocked() {
        return Math.abs(currentHoodTarget - HOOD_SMALL_BALL_POS) < 0.05;
    }

    public boolean isReadyToLaunch() {
        if (!isBallStagedAtTop() || ballQueue.isEmpty()) {
            return false;
        }

        BallType staged = ballQueue.peekFirst();
        if (staged == BallType.YELLOW_BALL) {
            return isOverCenterLocked();
        } else {
            return !isOverCenterLocked();
        }
    }

    public void setOverCenterLocked(boolean locked) {
        setHoodPosition(locked ? HOOD_SMALL_BALL_POS : HOOD_LARGE_BALL_POS);
    }

    public int getQueuedBallCount() {
        return ballQueue.size();
    }

    public void clearQueue() {
        ballQueue.clear();
        entryWasTriggered = false;
        topWasTriggered = false;
        sumR = 0f;
        sumG = 0f;
        sumB = 0f;
        sampleCount = 0;
        lastIdentifiedBall = BallType.UNKNOWN;
    }
}
