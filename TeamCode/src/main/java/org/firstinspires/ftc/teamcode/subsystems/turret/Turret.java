package org.firstinspires.ftc.teamcode.subsystems.turret;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.VelocityCompensationCalculator;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    public enum Mode {
        DISABLED,
        AUTO_AIM,
        MANUAL_ANGLE,
        CENTER
    }

    NextServo turretServoF = new NextServo(RobotConfig.turretServoFName, 0.0025, RobotConfig.turretServoFDirection);
    NextServo turretServoB = new NextServo(RobotConfig.turretServoBName, 0.0025, RobotConfig.turretServoBDirection);

    private Mode mode = Mode.AUTO_AIM;

    public static double turretOffsetRad = 0;
    public static double MIN_TURRET_ANGLE = Math.toRadians(-157);
    public static double MAX_TURRET_ANGLE = Math.toRadians(157);

    double[] angleValues = new double[]{Math.toRadians(-90), Math.toRadians(90)};
    double[] servoPositions = new double[]{0.78, 0.22}; // TODO: tune

    public double MIN_SERVO_POS = Math.min(getServoPosFromAngle(MIN_TURRET_ANGLE), getServoPosFromAngle(MAX_TURRET_ANGLE));
    public double MAX_SERVO_POS = Math.max(getServoPosFromAngle(MIN_TURRET_ANGLE), getServoPosFromAngle(MAX_TURRET_ANGLE));

    public Turret() {
    }

    public Turret(Robot robot) {
        this();
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Command setModeCommand(Mode mode) {
        return instant(() -> setMode(mode));
    }

    public double getAngleFromServoPos(double servoPos) {
        return (servoPos - servoPositions[0]) *
                (angleValues[1] - angleValues[0]) / (servoPositions[1] - servoPositions[0])
                + angleValues[0];
    }

    public double getServoPosFromAngle(double angleRad) {
        return (angleRad - angleValues[0]) *
                (servoPositions[1] - servoPositions[0]) / (angleValues[1] - angleValues[0])
                + servoPositions[0];
    }

    public double getTargetPosition() {
        return turretServoF.getPosition();
    }

    /**
     * @return The target angle of the turret in radians. Note that counter-clockwise is positive with 0 being straight forward.
     */
    public double getTargetAngle() {
        double currentPos = getTargetPosition();
        return getAngleFromServoPos(currentPos);
    }

    public double getCurrentAngle() {
        return getAngleFromServoPos(turretServoF.getPosition());
    }

    public void setServoPosition(double pos) {
        double clippedPos = Range.clip(pos, MIN_SERVO_POS, MAX_SERVO_POS);
        turretServoF.setPosition(clippedPos);
        turretServoB.setPosition(clippedPos);
    }

    public void setTurretAngle(double angleRad) {
        setMode(Mode.MANUAL_ANGLE);
        setServoPosition(getServoPosFromAngle(angleRad + turretOffsetRad));
    }

    public Command setTurretAngleCommand(double angleRad) {
        return instant(() -> setTurretAngle(angleRad));
    }

    public double getTurretOffsetRad() {
        return turretOffsetRad;
    }

    public void adjustOffset(double deltaRad) {
        turretOffsetRad += deltaRad;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case AUTO_AIM:
                double calculatedAngle = VelocityCompensationCalculator.getTurretAngle();
                setServoPosition(getServoPosFromAngle(calculatedAngle + turretOffsetRad));
                break;
            case CENTER:
                setServoPosition(getServoPosFromAngle(0 + turretOffsetRad));
                break;
            case MANUAL_ANGLE:
            case DISABLED:
                break;
        }
    }
}
