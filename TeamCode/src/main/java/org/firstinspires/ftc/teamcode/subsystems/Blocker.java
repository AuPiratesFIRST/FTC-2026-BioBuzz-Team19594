package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.Mechanism;

/**
 * Blocker Subsystem.
 * Controls the Blocker mechanism servo to BLOCK or UNBLOCK.
 */
@Config
public class Blocker implements Mechanism {

    public enum State {
        BLOCKED,
        UNBLOCKED
    }

    public static double BLOCKED_POSITION = 0.0;
    public static double UNBLOCKED_POSITION = 1.0;

    // Aliases for compatibility
    public static double CLOSED_POSITION = BLOCKED_POSITION;
    public static double OPEN_POSITION = UNBLOCKED_POSITION;

    private final Servo blockerServo;
    private State state = State.BLOCKED;

    public Blocker() {
        this((HardwareMap) null);
    }

    public Blocker(Robot robot) {
        this(robot != null ? robot.hardwareMap : null);
    }

    public Blocker(HardwareMap hardwareMap) {
        Servo temp = null;
        if (hardwareMap != null) {
            try {
                temp = hardwareMap.get(Servo.class, "blocker");
            } catch (Exception e1) {
                try {
                    temp = hardwareMap.get(Servo.class, "blockerServo");
                } catch (Exception e2) {
                    try {
                        temp = hardwareMap.get(Servo.class, "blocker_servo");
                    } catch (Exception ignored) {}
                }
            }
        }
        this.blockerServo = temp;
        block();
    }

    public State getState() {
        return state;
    }

    public boolean isBlocked() {
        return state == State.BLOCKED;
    }

    public boolean isUnblocked() {
        return state == State.UNBLOCKED;
    }

    // Aliases for compatibility
    public boolean isClosed() {
        return isBlocked();
    }

    public boolean isOpen() {
        return isUnblocked();
    }

    public void block() {
        state = State.BLOCKED;
        setPosition(BLOCKED_POSITION);
    }

    public void unblock() {
        state = State.UNBLOCKED;
        setPosition(UNBLOCKED_POSITION);
    }

    // Aliases for compatibility
    public void close() {
        block();
    }

    public void open() {
        unblock();
    }

    public void toggle() {
        if (isBlocked()) {
            unblock();
        } else {
            block();
        }
    }

    public void setPosition(double position) {
        if (blockerServo != null) {
            blockerServo.setPosition(Range.clip(position, 0.0, 1.0));
        }
    }

    public double getPosition() {
        return blockerServo != null ? blockerServo.getPosition() : (isBlocked() ? BLOCKED_POSITION : UNBLOCKED_POSITION);
    }

    public Command blockCommand() {
        return instant(this::block);
    }

    public Command unblockCommand() {
        return instant(this::unblock);
    }

    public Command toggleCommand() {
        return instant(this::toggle);
    }

    @Override
    public void periodic() {
        // Subsystem periodic update
    }
}
