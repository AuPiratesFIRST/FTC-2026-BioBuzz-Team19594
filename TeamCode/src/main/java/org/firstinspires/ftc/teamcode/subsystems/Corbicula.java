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
 * Corbicula Subsystem.
 * Controls the Corbicula mechanism servo to move UP or DOWN.
 */
@Config
public class Corbicula implements Mechanism {

    public enum State {
        UP,
        DOWN
    }

    public static double UP_POSITION = 0.8;
    public static double DOWN_POSITION = 0.2;

    private final Servo corbiculaServo;
    private State state = State.DOWN;

    public Corbicula() {
        this((HardwareMap) null);
    }

    public Corbicula(Robot robot) {
        this(robot != null ? robot.hardwareMap : null);
    }

    public Corbicula(HardwareMap hardwareMap) {
        Servo temp = null;
        if (hardwareMap != null) {
            try {
                temp = hardwareMap.get(Servo.class, "corbicula");
            } catch (Exception e1) {
                try {
                    temp = hardwareMap.get(Servo.class, "corbiculaServo");
                } catch (Exception e2) {
                    try {
                        temp = hardwareMap.get(Servo.class, "corbicula_servo");
                    } catch (Exception ignored) {}
                }
            }
        }
        this.corbiculaServo = temp;
        down();
    }

    public State getState() {
        return state;
    }

    public boolean isUp() {
        return state == State.UP;
    }

    public boolean isDown() {
        return state == State.DOWN;
    }

    public void up() {
        state = State.UP;
        setPosition(UP_POSITION);
    }

    public void down() {
        state = State.DOWN;
        setPosition(DOWN_POSITION);
    }

    public void toggle() {
        if (state == State.UP) {
            down();
        } else {
            up();
        }
    }

    public void setPosition(double position) {
        if (corbiculaServo != null) {
            corbiculaServo.setPosition(Range.clip(position, 0.0, 1.0));
        }
    }

    public double getPosition() {
        return corbiculaServo != null ? corbiculaServo.getPosition() : (state == State.UP ? UP_POSITION : DOWN_POSITION);
    }

    public Command upCommand() {
        return instant(this::up);
    }

    public Command downCommand() {
        return instant(this::down);
    }

    public Command toggleCommand() {
        return instant(this::toggle);
    }

    @Override
    public void periodic() {
        // Subsystem periodic update
    }
}
