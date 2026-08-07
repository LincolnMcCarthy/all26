package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

/**
 * List of N things.
 */
public class FixedList<N extends Num, T> {
    private final List<T> items;

    public FixedList(Nat<N> n, List<T> items) {
        if (items.size() != n.getNum())
            throw new IllegalArgumentException();
        this.items = items;
    }

    public FixedList(Nat<N> n) {
        items = new ArrayList<>();
        for (int i = 0; i < n.getNum(); ++i) {
            set(i, null);
        }
    }

    public T get(int i) {
        return items.get(i);
    }

    public void set(int i, T item) {
        items.set(i, item);
    }
}
