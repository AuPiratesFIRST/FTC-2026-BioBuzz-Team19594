package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);

        // Schedule all subsystem periodic tasks
        schedule(
//                robot.drivetrain.periodic(),
//                robot.vision.periodic()
                // robot.intake.periodic(), 
                // robot.flywheel.periodic()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    @Override
    public void loop() {
        Scheduler.execute();
        robot.telemetry.update();
    }
}
