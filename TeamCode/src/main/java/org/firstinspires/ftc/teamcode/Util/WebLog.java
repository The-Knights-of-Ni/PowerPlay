package org.firstinspires.ftc.teamcode.Util;

public class WebLog {
    public enum LogSeverity {
        VERBOSE,
        DEBUG,
        INFO,
        WARNING,
        ERROR
    }
    public String TAG;
    public String message;
    public LogSeverity severity;

    public WebLog(String tag, String message, LogSeverity severity) {
        this.TAG = tag;
        this.message = message;
        this.severity = severity;
    }
}
