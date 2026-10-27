package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.TileCoordinate;

public class DriveSubsystem {
    // Shared static pose to hand off location from Auto to TeleOp
    private static Pose poseTransfer = Pose.zero();

    public final Follower follower;
    private final Telemetry telemetry;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public DriveSubsystem(Robot robot) {
        this.telemetry = robot.telemetry;
        this.follower = Constants.create(robot.hardwareMap);

        frontLeft = robot.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = robot.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = robot.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = robot.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
    }

    public void usePreviousStartingPose() {
        follower.setPose(poseTransfer);
    }

    public void arcadeDrive(double forward, double strafe, double turn, boolean robotCentric) {
        follower.manual(forward, strafe, turn);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public TileCoordinate getCurrentPosition() {
        Pose pose = follower.pose();
        return new TileCoordinate(pose.x(), pose.y());
    }

    public double getHeading() {
        return follower.pose().heading();
    }

    public void setPosition(TileCoordinate pos) {
        follower.setPose(new Pose(pos.getX(), pos.getY(), follower.pose().heading()));
    }

    public void setHeading(double heading) {
        follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), heading));
    }

    public double getAngularVelocity() {
        // Approximate angular velocity if not available directly
        return 0; // Should be replaced with real IMU data if possible
    }

    public Command periodic() {
        return infinite(() -> {
            follower.update();
            poseTransfer = follower.pose();
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        });
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }
}
