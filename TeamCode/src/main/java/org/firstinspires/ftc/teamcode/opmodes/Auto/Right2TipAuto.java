package org.firstinspires.ftc.teamcode.opmodes.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.vision.AprilTagNavigator;

/**
 * Smart Autonomous: "Right 2 Tip Auto" with Vision Tip Verification.
 *
 * Uses AprilTag height and vertical motion tracking to verify if the goal has tipped.
 * If tipped: Proceeds immediately without wasting wait time.
 * If not tipped (missed shot): Runs adaptive fallback intake to re-collect pollen and re-attempt shoot.
 */
@Autonomous(name = "BioBuzz: Right 2 Tip Auto (Vision Smart)", group = "Competition")
public class Right2TipAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.3, 6.15, 180);
    private final Pose cornerPose = poseFactory.of(10.5, 6.15, 180);
    private final Path intakeCorner = line(start, cornerPose).constant(cornerPose);

    private final Pose shootByFlower = poseFactory.of(62.5, 132.5, 270);
    private final Path toSecondShoot = curve(
            cornerPose,
            poseFactory.of(35, 24, 0),
            poseFactory.of(6, 112, 0),
            shootByFlower
    ).heading(
            Interpolator.piecewise()
                    .until(0.2, Interpolator.constant(cornerPose))
                    .until(1.0, Interpolator.constant(shootByFlower))
    );

    private final Pose flowerBackup = poseFactory.of(46.5, 118, 90);
    private final Pose flowerIntake = poseFactory.of(46.5, 125.5, 90);
    private final Path intakeFlowerLittleBackup = line(shootByFlower, flowerBackup).heading(
            Interpolator.piecewise()
                    .until(0.5, Interpolator.constant(shootByFlower))
                    .until(1.0, Interpolator.linear(shootByFlower, flowerBackup))
    );
    private final Path intakeFlowerForward = line(flowerBackup, flowerIntake).constant(flowerIntake);
    private final Path intakeFlowerBigBackup = line(flowerIntake, flowerBackup).constant(flowerBackup);

    private final Path toThirdShoot = curve(
            flowerIntake,
            poseFactory.of(50, 100, 0),
            shootByFlower).heading(
            Interpolator.piecewise()
                    .until(0.5, Interpolator.constant(flowerIntake))
                    .until(1.0, Interpolator.constant(shootByFlower))
    );

    private final Pose parkPose = poseFactory.of(14, 94, 0);
    private final Path park = curve(
            shootByFlower,
            poseFactory.of(50, 97, 0),
            parkPose
    ).tangent();

    @Override
    public void init() {
        super.init();

        // Enable auto-aiming turret and flywheel velocity mode
        robot.turret.setMode(Turret.Mode.AUTO_AIM);
        robot.flywheel.setMode(Flywheel.Mode.VELOCITY);
    }

    /**
     * Checks if the goal target is down or actively tipping down.
     */
    private boolean isGoalDownOrTipping() {
        if (robot.vision == null) return false;
        AprilTagNavigator nav = robot.vision.getAprilTagNavigator();
        return nav.isTargetDown() || nav.getVerticalDirection() == AprilTagNavigator.VerticalDirection.MOVING_DOWN;
    }

    /**
     * Smart Command: Checks AprilTag vision to verify if the goal target has tipped down or is moving down.
     * Races vision tip detection against a max timeout (1000ms) to avoid stalling.
     */
    private Command verifyTipOrTimeout() {
        return race(
                waitUntil(this::isGoalDownOrTipping),
                waitMs(1000)
        );
    }

    private Command say(String text) {
        return instant(() -> telemetry.speak(text));
    }

    @Override
    public void start() {
        robot.drivetrain.follower.setPose(start);
        robot.drivetrain.follower.update();

        schedule(
                sequential(
                        say("Start Right 2 Tip Auto"),
                        instant(() -> {
                            robot.flywheel.setMode(Flywheel.Mode.VELOCITY);
                            robot.turret.setMode(Turret.Mode.AUTO_AIM);
                        }),
                        waitUntil(() -> robot.flywheel.isReady()),
                        say("Shooting First Shot"),
                        robot.intake.on(),
                        waitMs(300),
                        robot.intake.off(),

                        // Move to corner intake position
                        parallel(
                                robot.drivetrain.followPath(intakeCorner),
                                sequential(
                                        waitUntil(() -> robot.drivetrain.follower.completion() >= 0.1),
                                        robot.intake.on()
                                )
                        ),
                        waitMs(500),
                        robot.intake.off(),

                        // Move to second shoot location by flower
                        parallel(
                                robot.drivetrain.followPath(toSecondShoot),
                                sequential(
                                        waitUntil(() -> robot.drivetrain.follower.completion() >= 0.1),
                                        say("Heading to second shoot")
                                )
                        ),
                        waitUntil(() -> !robot.drivetrain.follower.isBusy()),

                        // Second shoot execution
                        say("Firing second shoot"),
                        robot.intake.on(),
                        waitMs(500),
                        robot.intake.off(),

                        // SMART VISION CHECK: Is goal down or moving down?
                        say("Checking goal tip status"),
                        verifyTipOrTimeout(),

                        // Adaptive Branch based on Vision Tip Status
                        conditional(
                                this::isGoalDownOrTipping,
                                // BRANCH A: Goal is DOWN or MOVING DOWN -> Continue to next side/flower immediately without wasting balls or time
                                sequential(
                                        say("Goal tipped or falling! Continuing to next target"),
                                        robot.intake.on(),
                                        robot.drivetrain.followPath(intakeFlowerLittleBackup),
                                        robot.drivetrain.followPath(intakeFlowerForward).raceWith(waitMs(1500)),
                                        robot.drivetrain.followPath(toThirdShoot),
                                        waitUntil(() -> !robot.drivetrain.follower.isBusy()),
                                        robot.intake.on(),
                                        waitMs(500),
                                        robot.intake.off()
                                ),
                                // BRANCH B: Goal STILL UP -> Re-attempt pollen collect and shoot more to tip it
                                sequential(
                                        say("Goal still UP. Shooting more pollen to tip it"),
                                        robot.intake.on(),
                                        robot.drivetrain.followPath(intakeFlowerLittleBackup),
                                        robot.drivetrain.followPath(intakeFlowerForward).raceWith(waitMs(1500)),
                                        robot.drivetrain.followPath(intakeFlowerBigBackup),
                                        robot.drivetrain.followPath(intakeFlowerForward).raceWith(waitMs(1500)),
                                        robot.drivetrain.followPath(toThirdShoot),
                                        waitUntil(() -> !robot.drivetrain.follower.isBusy()),
                                        say("Firing extra shot"),
                                        robot.intake.on(),
                                        waitMs(500),
                                        robot.intake.off()
                                )
                        ),

                        // Park
                        robot.drivetrain.followPath(park),
                        say("Parked and finished")
                )
        );
    }
}
