package org.firstinspires.ftc.teamcode.Util;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class TimeUtil {
    private static long modeStartTimeNanoseconds = 0;

    /**
     * This method is called at the start of a competition mode to set the mode start timestamp so
     * that getModeElapsedTime can calculate the mode elapsed time.
     */
    public static void recordModeStartTime() {
        modeStartTimeNanoseconds = System.nanoTime();
    }

    /**
     * This method returns the competition mode elapsed time by subtracting mode start time from the
     * current time. If this method is called before the competition mode is started, the system
     * elapsed time is returned instead.
     *
     * @return mode elapsed time in seconds.
     */
    public static double getModeElapsedTime() {
        return (System.nanoTime() - modeStartTimeNanoseconds) / 1000000000.0;
    }

    /**
     * This method returns the current time in seconds with nano-second precision.
     *
     * @return current time in seconds.
     */
    public static double getCurrentTime() {
        return System.nanoTime() / 1000000000.0;
    }

    /**
     * This method returns the current time in milliseconds.
     *
     * @return current time in milliseconds.
     */
    public static long getCurrentTimeMilliseconds() {
        return System.currentTimeMillis();
    }

    /**
     * This method returns the current time in nanoseconds.
     *
     * @return current time in nanoseconds.
     */
    public static long getCurrentTimeNanoseconds() {
        return System.nanoTime();
    }

    /**
     * This method returns the current time stamp with the specified format.
     *
     * @param format specifies the time stamp format.
     * @return current time stamp string with the specified format.
     */
    public static String getTimestamp(String format) {
        SimpleDateFormat dateFormat = new SimpleDateFormat(format, Locale.US);
        return dateFormat.format(new Date());
    }

    /**
     * This method returns the current time stamp with the default format.
     *
     * @return current time stamp string with the default format.
     */
    public static String getTimestamp() {
        return getTimestamp("yyyyMMdd@HHmmss");
    }

    /**
     * This method puts the current thread to sleep for the given time in msec. It handles
     * InterruptException where it recalculates the remaining time and calls sleep again repeatedly
     * until the specified sleep time has past.
     *
     * @param time specifies sleep time in milliseconds.
     */
    public static void sleep(long time) {
        long wakeupTime = System.currentTimeMillis() + time;

        while (time > 0) {
            try {
                Thread.sleep(time);
                break;
            } catch (InterruptedException e) {
                time = wakeupTime - System.currentTimeMillis();
            }
        }
    }
}
