package dev.frozenmilk.dairy.core;

import java.lang.annotation.*;

public class FeatureRegistrar {

    @Target(ElementType.TYPE)
    @Retention(RetentionPolicy.RUNTIME)
    public @interface LogDependencyResolutionExceptions {}

    public static boolean isOpModeRunning() {
        return true;
    }
}
