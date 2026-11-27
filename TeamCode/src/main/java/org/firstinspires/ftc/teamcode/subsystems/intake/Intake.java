package org.firstinspires.ftc.teamcode.subsystems.intake;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.nextftc.robot.Mechanism;

@Config
public class Intake implements Mechanism {
    public static boolean logCurrent = false;

    public static class IntakeShaft {
        public final DcMotorEx motor;

        public IntakeShaft(DcMotorEx motor) {
            this.motor = motor;
            if (motor != null) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public Command setPower(double power) {
            return com.pedropathing.ivy.commands.Commands.instant(() -> {
                if (motor != null) {
                    motor.setPower(power);
                }
            });
        }

        public Command on() {
            return setPower(1);
        }

        public Command off() {
            return setPower(0);
        }

        public Command reverse() {
            return setPower(-1);
        }

        public boolean isOn() {
            return motor != null && motor.getPower() > 0;
        }

        public boolean isReversing() {
            return motor != null && motor.getPower() < 0;
        }

        public boolean isOff() {
            return motor == null || motor.getPower() == 0;
        }

        public double getCurrent() {
            return motor != null ? motor.getCurrent(CurrentUnit.AMPS) : 0;
        }
    }

    public final IntakeShaft lower;
    public final IntakeShaft upper;

    public Intake() {
        this((HardwareMap) null);
    }

    public Intake(Robot robot) {
        this(robot != null ? robot.hardwareMap : null);
    }

    public Intake(HardwareMap hardwareMap) {
        if (hardwareMap != null) {
            lower = new IntakeShaft(hardwareMap.get(DcMotorEx.class, "lowerIntake"));
            upper = new IntakeShaft(hardwareMap.get(DcMotorEx.class, "upperIntake"));
        } else {
            lower = new IntakeShaft(null);
            upper = new IntakeShaft(null);
        }
    }

    public Command on() {
        return sequential(upper.on(), lower.on());
    }

    public Command off() {
        return sequential(upper.off(), lower.off());
    }

    public Command reverse() {
        return sequential(upper.reverse(), lower.reverse());
    }

    public Command toggle() {
        return conditional(() -> upper.isOff() && lower.isOff(), on(), off());
    }

    public Command command() {
        return com.pedropathing.ivy.commands.Commands.infinite(this::periodic);
    }

    @Override
    public void periodic() {
        // Subsystem periodic loop execution
    }
}
