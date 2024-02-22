/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * linearInterpolator - given a table of x and y values, will interpolate values
 * of y between known values of x using linear interpolation.
 * 
 * Usage: private double[][] data = { {1.0, 10.0}, {3.0, 31.0}, {10,100} };
 * private linearInterpolator lt = new linearInterpolator(data);
 * 
 * double y = lt.getInterpolatedValue(1.5); // returns 15.25
 */
public class LinearInterpolator {

    // https://therevisionist.org/software-engineering/java/tutorials/passing-2d-arrays/
    private double[][] table;
    private boolean initialized = false;

    /**
     * create linearInterpolator class
     * 
     * @param data, a table of x -> y mappings to be interpolated
     */
    public LinearInterpolator(double[][] data) {
        build_table(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Build the internal representation of the table data.
     * 
     * @param data a table of data to be interpolated
     */
    private void build_table(double[][] data) {
        int rows = data.length;
        if (rows < 1) {
            DriverStation.reportError("ERROR: linearInterpolator needs at least one data point.", false);
            return;
        }
        int cols = data[0].length;
        if (cols != 2) {
            DriverStation.reportError("ERROR: linearInterpolator number of columns should be 2", false);
            return;
        }

        table = new double[rows][cols];
        for (int x = 0; x < data.length; x++) {
            for (int y = 0; y < data[x].length; y++) {
                table[x][y] = data[x][y];
            }
        }
        Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        initialized = true;
    }

    /**
     * getInterpolatedValue() - return the interpolated value of y given x.
     * 
     * If the value of x is in the table, that value is returned.
     * 
     * If the value of x is not in the table, the closest two values of x are chosen
     * and the value of y returned is interpolated between the corresponding y
     * values.
     * 
     * If the value of x is greater than max x value, the corresponding value of y
     * for max x is returned. If the value of x is less than the min x value, the
     * corresponding value of y for the min x is returned.
     *
     * @param x, the value of x to get an interpolated y value for
     * @return the linear interpolated value y
     */
    public double getInterpolatedValue(double x) {

        if (!initialized) {
            DriverStation.reportError("ERROR: linearInterpolator not initialized", false);
            return 0.0;
        }

        // NOTE: this uses linear search, for larger tables (>5), binary search would be
        // faster
        int index = 0;
        for (index = 0; index < table.length; index++) {
            if (table[index][0] >= x) {
                break;
            }
        }

        // System.out.println("index of " + x + " is " + index);

        if (index >= table.length) {
            return table[table.length - 1][1];
        }

        double high_y = table[index][1];
        double high_x = table[index][0];
        if ((high_x == x) || (index == 0)) {
            return high_y;
        }
        double low_y = table[index - 1][1];
        double low_x = table[index - 1][0];

        return (low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x));
    }
}