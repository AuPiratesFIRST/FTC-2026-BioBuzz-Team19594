package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DualSensorIntakeSubsystem;

/**
 * Diagnostic & Performance Test OpMode for DualSensorIntakeSubsystem & High-Speed Control Loop.
 *
 * Tests:
 * 1. Control Loop Benchmarking (Hz and ms per loop)
 * 2. Entry & Top Sensor Proximity & Color Identification
 * 3. 4-Ball FIFO Queue Tracking & Over-Center Hood Servo Actuation
 * 4. Manual Over-Center Linkage Override & Queue Reset
 * 5. Real-time Sensor & Color Diagnostics
 *
 * Gamepad 1 Controls:
 * - A: Manual Over-Center Linkage LOCK (Small / Yellow Pollen position)
 * - B: Manual Over-Center Linkage RETRACT (Large / Alliance ball position)
 * - X: Clear FIFO Queue & Reset Sensors
 */
@TeleOp(name = "Test: Dual Sensor Intake & Loop Benchmark", group = "Diagnostics")
public class DualSensorIntakeTestOpMode extends RobotOpMode {

    private final ElapsedTime loopTimer = new ElapsedTime();

    private long loopCount = 0;
    private double minLoopTimeMs = Double.MAX_VALUE;
    private double maxLoopTimeMs = 0.0;
    private double avgLoopTimeMs = 0.0;
    private double currentHz = 0.0;

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized Dual Sensor Intake Test OpMode");
    }

    @Override
    public void start() {
        loopTimer.reset();
        loopCount = 0;
        minLoopTimeMs = Double.MAX_VALUE;
        maxLoopTimeMs = 0.0;
        avgLoopTimeMs = 0.0;
    }

    @Override
    public void loop() {
        // Measure exact loop time
        double loopTimeMs = loopTimer.milliseconds();
        loopTimer.reset();

        if (loopTimeMs > 0 && loopTimeMs < 1000.0) {
            minLoopTimeMs = Math.min(minLoopTimeMs, loopTimeMs);
            maxLoopTimeMs = Math.max(maxLoopTimeMs, loopTimeMs);
            loopCount++;

            // Exponential moving average for smooth Hz readout
            avgLoopTimeMs = (avgLoopTimeMs == 0.0) ? loopTimeMs : (avgLoopTimeMs * 0.92 + loopTimeMs * 0.08);
            currentHz = 1000.0 / avgLoopTimeMs;
        }

        // 1. Update DualSensorIntake Subsystem FIRST
        if (robot.dualSensorIntake != null) {
            robot.dualSensorIntake.update();

            // Gamepad Overrides
            if (gamepad1.a) {
                robot.dualSensorIntake.setOverCenterLocked(true);
            } else if (gamepad1.b) {
                robot.dualSensorIntake.setOverCenterLocked(false);
            }

            if (gamepad1.x) {
                robot.dualSensorIntake.clearQueue();
            }
        }

        // ==================== TELEMETRY & DIAGNOSTICS ====================
        telemetry.addLine("=== CONTROL LOOP BENCHMARK ===");
        telemetry.addData("Loop Frequency", "%.1f Hz", currentHz);
        telemetry.addData("Loop Duration", "%.2f ms/loop (Avg)", avgLoopTimeMs);
        telemetry.addData("Min / Max Loop Time", "%.2f ms / %.2f ms",
                minLoopTimeMs == Double.MAX_VALUE ? 0.0 : minLoopTimeMs, maxLoopTimeMs);
        telemetry.addData("Total Loop Cycles", loopCount);

        telemetry.addLine("\n=== DUAL SENSOR INTAKE DIAGNOSTICS ===");
        if (robot.dualSensorIntake != null) {
            DualSensorIntakeSubsystem dsi = robot.dualSensorIntake;
            telemetry.addData("Entry Dist / Triggered", "%.2f in | %b", dsi.getEntryDistance(), dsi.isEntryTriggered());
            telemetry.addData("Top Dist / Staged", "%.2f in | %b", dsi.getTopDistance(), dsi.isBallStagedAtTop());
            telemetry.addData("Entry Color Reading", dsi.getEntryRawColorString());
            telemetry.addData("G/R Ratio", "%.2f", dsi.getLastGRRatio());
            telemetry.addData("Detected Color", dsi.getDetectedEntryColor() != null ? dsi.getDetectedEntryColor().name() : "NONE");
            telemetry.addData("Queued Ball Count", "%d / %d", dsi.getQueuedBallCount(), DualSensorIntakeSubsystem.MAX_QUEUE_CAPACITY);
            telemetry.addData("FIFO Pattern List", dsi.getQueuePattern().toString());
            telemetry.addData("Staged Ball", dsi.getStagedBall() != null ? dsi.getStagedBall().name() : "NONE");
            telemetry.addData("Over-Center Locked", dsi.isOverCenterLocked());
            telemetry.addData("Ready To Launch", dsi.isReadyToLaunch());
        } else {
            telemetry.addData("DualSensorIntake", "NOT INITIALIZED (Verify 'entry_sensor', 'top_sensor', 'hood_servo' in Driver Station Hardware Config)");
        }

        telemetry.addLine("\n=== GAMEPAD CONTROLS ===");
        telemetry.addLine("A: Force Lock Hood (Small/Pollen)");
        telemetry.addLine("B: Force Retract Hood (Large/Alliance)");
        telemetry.addLine("X: Reset FIFO Ball Queue");

        // Execute scheduler tasks and flush telemetry to Driver Station LAST
        super.loop();
    }
}
