package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import java.util.Collections;
import java.util.Set;

public class TeleOpDriveCommand implements Command {
    private final Drivetrain drive;
    private final Gamepad gamepad;

    public TeleOpDriveCommand(Drivetrain drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        // This command runs every loop unless interrupted by AutoIntake
        drive.arcadeDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true // Set to false for Field Centric
        );
    }

    @Override
    public boolean done() {
        return false; // Driver control never "finishes" on its own
    }

    @Override
    public Set<Object> requirements() {
        return Collections.singleton(drive);
    }

    @Override
    public int priority() {
        return 0;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return null;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return null;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return null;
    }

    @Override
    public void start() {

    }

    @Override
    public void end(EndCondition endCondition) {
        drive.arcadeDrive(0,0,0, true);
    }
}