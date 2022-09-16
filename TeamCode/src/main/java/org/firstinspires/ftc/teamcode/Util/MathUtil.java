package org.firstinspires.ftc.teamcode.Util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

public class MathUtil {
    /**
     * This method calculates the modulo of two numbers. Unlike the <code>%</code> operator, this
     * returns a number in the range [0, b). For some reason, in Java, the <code>%</code> operator
     * actually does remainder, which means the result is in the range (-b, b).
     *
     * @param a specifies the dividend.
     * @param b specifies the divisor.
     * @return the modulo in the range [0, b)
     */
    public static double modulo(double a, double b) {
        return ((a % b) + b) % b;
    } // modulo

    /**
     * This method sums an array of numbers.
     *
     * @param nums specifies the array of numbers to be summed.
     * @return sum of the numbers.
     */
    public static double sum(double... nums) {
        double total = 0.0;

        for (double num : nums) {
            total += num;
        }

        return total;
    } // sum

    /**
     * This method calculates and returns the median of the numbers in the given array.
     *
     * @param num specifies the number array.
     * @return median of numbers in the array.
     */
    public static double median(double... num) {
        double m = 0.0;

        if (num.length > 0) {
            double[] nums = num.clone();

            Arrays.sort(nums);
            if (nums.length % 2 == 0) {
                m = average(nums[(nums.length / 2) - 1], nums[nums.length / 2]);
            } else {
                m = nums[nums.length / 2];
            }
        }

        return m;
    } // median

    /**
     * This method calculates and returns the average of the numbers in the given array.
     *
     * @param nums specifies the number array.
     * @return average of all numbers in the array. If the array is empty, return 0.
     */
    public static double average(double... nums) {
        return nums.length == 0 ? 0.0 : sum(nums) / nums.length;
    }

    /**
     * This method calculates the magnitudes of the given array of numbers.
     *
     * @param nums specifies the number array.
     * @return magnitude of all numbers in the array.
     */
    public static double magnitude(double... nums) {
        double total = 0.0;

        for (double num : nums) {
            total += num * num;
        }

        return Math.sqrt(total);
        // return Math.sqrt(Arrays.stream(nums).map(e -> e*e).sum());
    } // magnitude

    /**
     * This method returns the maximum magnitude of numbers in the specified array.
     *
     * @param nums specifies the number array.
     * @return maximum magnitude of the numbers in the array.
     */
    public static double maxMagnitude(double... nums) {
        double maxMag = Math.abs(nums[0]);

        for (double num : nums) {
            double magnitude = Math.abs(num);
            if (magnitude > maxMag) {
                maxMag = magnitude;
            }
        }

        return maxMag;
    } // maxMagnitude

    /**
     * This method returns a bit mask of the least significant set bit.
     *
     * @param data specifies the data to find the least significant set bit.
     * @return bit mask that has only the least significant set bit.
     */
    public static int leastSignificantSetBit(int data) {
        int bitMask = 0;

        if (data != 0) {
            bitMask = data & ~(data ^ -data);
        }

        return bitMask;
    }

    /**
     * This method returns the bit position of the least significant set bit of the given data.
     *
     * @param data specifies the data to determine its least significant set bit position.
     * @return 0-based least significant set bit position. -1 if no set bit.
     */
    public static int leastSignificantSetBitPosition(int data) {
        int pos = -1;

        if (data != 0) {
            for (int i = 0; ; i++) {
                if ((data & (1 << i)) != 0) {
                    pos = i;
                    break;
                }
            }
        }

        return pos;
    }

    /**
     * This method returns the bit position of the most significant set bit of the given data.
     *
     * @param data specifies the data to determine its most significant set bit position.
     * @return 0-based most significant set bit position. -1 if no set bit.
     */
    public static int mostSignificantSetBitPosition(int data) {
        int pos = -1;

        if (data != 0) {
            for (int i = 0; ; i++) {
                if ((data & (0x80000000 >> i)) != 0) {
                    pos = 31 - i;
                    break;
                }
            }
        }

        return pos;
    }

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0.
     *
     * @param nums specifies the number array.
     * @return normalized number array.
     */
    public static double[] normalize(double... nums) {
        double[] result = nums.clone();
        normalizeInPlace(result);

        return result;
    }

    /**
     * This method rounds a double to the nearest integer.
     *
     * @param num Number to round.
     * @return Rounded to the nearest integer.
     */
    public static int round(double num) {
        return (int) Math.floor(num + 0.5);
    }

    /**
     * This method sets a bitmask with the given bit positions.
     *
     * @param bitPositions specifies the bit positions to be set to 1. Bit positions are 0-based.
     * @return bit mask.
     */
    public static int setBitMask(int... bitPositions) {
        int bitMask = 0;

        for (int pos : bitPositions) {
            bitMask |= 1 << pos;
        }

        return bitMask;
    }

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0. If no
     * number exceeds the magnitude of 1.0, nothing will change, otherwise the original nums array
     * will be modified in place.
     *
     * @param nums specifies the number array.
     */
    public static void normalizeInPlace(double[] nums) {
        double maxMag = maxMagnitude(nums);

        if (maxMag > 1.0) {
            for (int i = 0; i < nums.length; i++) {
                nums[i] /= maxMag;
            }
        }
    }

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is
     *                  exclusive (low, high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high, boolean inclusive) {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high) {
        return inRange(value, low, high, true);
    }

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is
     *                  exclusive (low,high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high, boolean inclusive) {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high) {
        return inRange(value, low, high, true);
    }

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static int clipRange(int value, int lowLimit, int highLimit) {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static double clipRange(double value, double lowLimit, double highLimit) {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }

    /**
     * This method clips the given value to the range between -1.0 and 1.0.
     *
     * @param value specifies the value to be clipped
     * @return the result of the clipped value.
     */
    public static double clipRange(double value) {
        return clipRange(value, -1.0, 1.0);
    }

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static int scaleRange(
            int value, int lowSrcRange, int highSrcRange, int lowDstRange, int highDstRange) {
        return lowDstRange
                + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static double scaleRange(
            double value,
            double lowSrcRange,
            double highSrcRange,
            double lowDstRange,
            double highDstRange) {
        return lowDstRange
                + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }

    /**
     * This method checks if the given value is within the deadband range. If so, it returns 0.0 else
     * it returns the unchanged value.
     *
     * @param value    specifies the value to be checked.
     * @param deadband specifies the deadband zone.
     * @return the value 0.0 if within deadband, unaltered otherwise.
     */
    public static double applyDeadband(double value, double deadband) {
        return Math.abs(value) >= deadband ? value : 0.0;
    }

    /**
     * Convert a point from a polar coordinate system to a cartesian coordinate system.
     *
     * @param r     Magnitude of vector
     * @param theta Direction of vector, in degrees clockwise from 0 (+y)
     * @return Vector in a cartesian coordinate system representing the same point.
     */
    public static RealVector polarToCartesian(double r, double theta) {
        double thetaRad = Math.toRadians(theta);
        return MatrixUtils.createRealVector(
                new double[]{r * Math.sin(thetaRad), r * Math.cos(thetaRad)});
    }

    /**
     * Rotate a point counter-clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCCW(RealVector vector, double angle) {
        return createCCWRotationMatrix(angle).operate(vector);
    }

    /**
     * Rotate a point clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCW(RealVector vector, double angle) {
        return createCWRotationMatrix(angle).operate(vector);
    }

    /**
     * Create a rotation matrix that will rotate a point counter-clockwise about the origin by a
     * specific number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a counter-clockwise rotation by <code>angle</code>
     * degrees.
     */
    public static RealMatrix createCCWRotationMatrix(double angle) {
        double angleRad = Math.toRadians(angle);
        return MatrixUtils.createRealMatrix(
                new double[][]{
                        {Math.cos(angleRad), -Math.sin(angleRad)}, {Math.sin(angleRad), Math.cos(angleRad)}
                });
    }

    /**
     * Create a rotation matrix that will rotate a point clockwise about the origin by a specific
     * number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a clockwise rotation by <code>angle</code> degrees.
     */
    public static RealMatrix createCWRotationMatrix(double angle) {
        return createCCWRotationMatrix(angle).transpose();
    }
}
