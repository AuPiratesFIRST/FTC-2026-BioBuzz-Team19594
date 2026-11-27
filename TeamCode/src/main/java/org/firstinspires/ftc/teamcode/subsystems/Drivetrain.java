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

public class Drivetrain {
    // Shared static pose to hand off location from Auto to TeleOp
    private static Pose poseTransfer = Pose.zero();

    public final Follower follower;
    private final Telemetry telemetry;
    private final DcMotorEx leftFront, rightFront, leftRear, rightRear;

    public Drivetrain(Robot robot) {
        this.telemetry = robot.telemetry;
        this.follower = Constants.createFollower(robot.hardwareMap);

        leftFront = robot.hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = robot.hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = robot.hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = robot.hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public Follower getFollower() {
        return follower;
    }


    public Command periodic() {
        return infinite(() -> {
            follower.update();
            poseTransfer = follower.pose();

            // Update goal target selection and shoot-on-the-move velocity lead compensation
            org.firstinspires.ftc.teamcode.Constants.HivePoses.update(follower);
            org.firstinspires.ftc.teamcode.util.VelocityCompensationCalculator.update(follower);

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        });
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }
}