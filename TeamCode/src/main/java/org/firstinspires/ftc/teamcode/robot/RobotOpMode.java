package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTime = 0;
    public static double loopTimeMs = 0;
    public static double loopHz = 0;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);

        // Cap Driver Station telemetry transmission to 50ms so Bluetooth/Wi-Fi doesn't bottleneck the 200Hz loop
        telemetry.setMsTransmissionInterval(50);

        // Schedule all subsystem periodic Ivy commands together
        schedule(
                robot.drivetrain.periodic(),
                robot.vision.periodic(),
                robot.intake.command(),
                robot.flywheel.command(),
                robot.turret.command()
        );

        loopTimer.reset();
        lastLoopTime = loopTimer.seconds();
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    @Override
    public void loop() {
        double currentTime = loopTimer.seconds();
        double dt = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

        if (dt > 0) {
            loopTimeMs = dt * 1000.0;
            loopHz = 1.0 / dt;
        }

        Scheduler.execute();

        robot.telemetry.addData("Loop Rate", "%.1f ms (%.0f Hz)", loopTimeMs, loopHz);
        robot.telemetry.update();
    }
}
