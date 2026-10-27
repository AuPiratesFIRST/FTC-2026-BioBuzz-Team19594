package dev.frozenmilk.curdled.ez.bulkreads;

import java.lang.annotation.*;

public class BulkReads {

    @Target(ElementType.TYPE)
    @Retention(RetentionPolicy.RUNTIME)
    public @interface Attach {}
}
