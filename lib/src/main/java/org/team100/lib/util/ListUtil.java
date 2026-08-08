package org.team100.lib.util;

import java.util.List;

public class ListUtil {
    /** Throw if the list size is incorrect. */
    public static <T> void size(int n, List<T> list) {
        if (list.size() != n)
            throw new IllegalArgumentException();
    }
}
