package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class PoseMirror {
    // Official FTC field center line (141.5" x 141.5" field, center at 70.75)
    public static final double FIELD_CENTER = 70.75;

    public static final PoseFactory radiansFactory = PoseFactory.radians();
    public static final PoseFactory degreesFactory = PoseFactory.degrees();

    private static final PoseFactory blueMirrorFactory = PoseFactory.radians()
            .mirrorX(FIELD_CENTER)
            .mirrorY(FIELD_CENTER)
            .mapHeading(h -> h + Math.PI);

    public static Pose mirror(Pose pose) {
        return blueMirrorFactory.of(pose.x(), pose.y(), pose.heading());
    }
}
