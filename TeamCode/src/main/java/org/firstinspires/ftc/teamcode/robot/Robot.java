package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Corbicula;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlywheelShooter;
import org.firstinspires.ftc.teamcode.subsystems.scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sensors.DualSensorIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Drivetrain drivetrain;
    public DriveSubsystem driveSubsystem;
    public VisionSubsystem vision;
    public Intake intake;
    public Blocker blocker;
    public Corbicula corbicula;
    public Turret turret;
    public FlywheelShooter flywheel;
    public ScoringSubsystem scoring;
    public DualSensorIntakeSubsystem dualSensorIntake;

    public Robot() {
        this.turret = new Turret();
        this.flywheel = new FlywheelShooter();
    }

    public Robot(OpMode opMode) {
        this();
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        // Enable AUTO LynxModule bulk caching to drastically reduce I2C/CAN bus latency
        if (hardwareMap != null) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        this.drivetrain = new Drivetrain(this);
        this.driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        this.vision = new VisionSubsystem(this, driveSubsystem);
        this.intake = new Intake(this);
        this.blocker = new Blocker(this);
        this.corbicula = new Corbicula(this);
        this.scoring = new ScoringSubsystem(this);

        if (hardwareMap != null) {
            try {
                this.dualSensorIntake = new DualSensorIntakeSubsystem(
                        hardwareMap, Alliance.color == Alliance.RED
                );
            } catch (Exception ignored) {
                this.dualSensorIntake = null;
            }
        } else {
            this.dualSensorIntake = null;
        }
    }

    @Override
    public void periodic() {
        // NextFTC robot periodic loop
    }

    @Override
    @NotNull
    public Set<Mechanism> getMechanisms() {
        Set<Mechanism> mechanisms = new HashSet<>();
        if (turret != null) mechanisms.add(turret);
        if (flywheel != null) mechanisms.add(flywheel);
        if (blocker != null) mechanisms.add(blocker);
        if (corbicula != null) mechanisms.add(corbicula);
        return Collections.unmodifiableSet(mechanisms);
    }
}
