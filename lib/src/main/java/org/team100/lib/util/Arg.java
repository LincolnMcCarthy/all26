package org.team100.lib.util;

import java.util.function.Supplier;

public class Arg {
    public static void verify(Supplier<Boolean> r) {
        if (!r.get())
            throw new IllegalArgumentException();
    }
}
