package org.team100.lib.util;

import org.jfree.chart3d.data.xyz.XYZDataset;
import org.junit.jupiter.api.Test;
import org.team100.lib.trajectory.se3.TrajectorySE3ToVectorSeries;

/** example 3d chart */
public class Chart3dTest {

    @Test
    void test0() {
        XYZDataset<String> dataset = TrajectorySE3ToVectorSeries.data();
        ChartUtil3d.plot3dVectorSeries(dataset);
    }

}
