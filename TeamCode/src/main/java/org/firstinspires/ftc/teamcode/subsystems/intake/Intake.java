package org.firstinspires.ftc.teamcode.subsystems.intake;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Intake {
    public Intake(Robot robot) {}
    public Command on() { return instant(() -> {}); }
    public Command off() { return instant(() -> {}); }
    public Command periodic() { return instant(() -> {}); }
}
