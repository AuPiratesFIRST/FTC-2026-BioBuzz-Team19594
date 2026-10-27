package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelShooter;
import org.firstinspires.ftc.teamcode.subsystems.scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DualSensorIntakeSubsystem;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    
    public final Drivetrain drivetrain;
    public final DriveSubsystem driveSubsystem;
    public final VisionSubsystem vision;
    public final Intake intake;
    public final Blocker blocker;
    public final Turret turret;
    public final FlywheelShooter flywheel;
    public final ScoringSubsystem scoring;

    public Robot(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        this.drivetrain = new Drivetrain(this);
        this.driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        this.vision = new VisionSubsystem(this, driveSubsystem);
        this.intake = new Intake(this);
        this.blocker = new Blocker(this);
        this.turret = new Turret(this);
        this.flywheel = new FlywheelShooter(this);
        this.scoring = new ScoringSubsystem(this);

        DualSensorIntakeSubsystem dsi = null;
        try {
            dsi = new org.firstinspires.ftc.teamcode.subsystems.sensors.DualSensorIntakeSubsystem(
                    hardwareMap, Alliance.color == Alliance.RED
            );
        } catch (Exception ignored) {}
        this.dualSensorIntake = dsi;
    }
    public final org.firstinspires.ftc.teamcode.subsystems.sensors.DualSensorIntakeSubsystem dualSensorIntake;
}
