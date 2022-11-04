package org.firstinspires.ftc.teamcode.Subsystems.Web;

import android.util.JsonWriter;
import io.javalin.Javalin;
import org.firstinspires.ftc.teamcode.Util.WebLog;

import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;


public class Web {
    WebThreadData wtd = WebThreadData.getWtd();
    public Javalin app;
    int port;
    /** Version of the protocol upgrade (major.minor.patch) update major for protocol changes that cause the server to
     * be completely incompatible to previous version, minor should be used for functionality addition of modification
     * patch should be for bug fixes that affect the result
     */
    public static final String VERSION = "0.1.1";

    public Web(int port) {
        init();
        this.port = port;
    }

    /** Initializes and configures the app and registers the url handlers.*/
    private void init() {
        app = Javalin.create(config -> {
//            OpenApiConfiguration openApiConfiguration = new OpenApiConfiguration();
//            openApiConfiguration.getInfo().setTitle("Web Subsystem OpenAPI");
//            config.plugins.register(new OpenApiPlugin(openApiConfiguration));
//            config.plugins.register(new SwaggerPlugin(new SwaggerConfiguration()));
//            config.plugins.register(new ReDocPlugin(new ReDocConfiguration()));
        });
        app.get("/logs", ctx -> ctx.result(getLogs()));
        app.get("/", ctx -> ctx.result(
                "{\n" +
                "    \"version\": \"" + VERSION + "\"" +
                "\n}"));
        app.get("/robot-position", ctx -> ctx.result(getRobotPosition()));
    }


    public String writeJson(ArrayList<WebLog> logs) throws IOException {
        OutputStream output = new OutputStream() {
            private StringBuilder string = new StringBuilder();

            @Override
            public void write(int b) throws IOException {
                this.string.append((char) b);
            }

            @Override
            public String toString() {
                return this.string.toString();
            }
        };
        writeJsonStream(output, logs);
        return output.toString();
    }

    public void writeJsonStream(OutputStream out, ArrayList<WebLog> logs) throws IOException {
        JsonWriter writer = new JsonWriter(new OutputStreamWriter(out, StandardCharsets.UTF_8));
        writer.setIndent("    ");
        writeMessagesArray(writer, logs);
        writer.close();
    }

    public void writeMessagesArray(JsonWriter writer, ArrayList<WebLog> messages) throws IOException {
        writer.beginArray();
        for (WebLog log : messages) {
            writeMessage(writer, log);
        }
        writer.endArray();
    }

    public void writeMessage(JsonWriter writer, WebLog webLog) throws IOException {
        writer.beginObject();
        writer.name("tag").value(webLog.TAG);
        writer.name("message").value(webLog.message);
        writer.name("severity").value(webLog.severity.ordinal());
        writer.endObject();
    }


    public String getLogs() {
        StringBuilder json = new StringBuilder("{");
        ArrayList<WebLog> logs = wtd.getLogs();
        for (WebLog log: logs) {
            json.append("\n{\n" + "\"tag\": ").append(log.TAG).append(",\n\"message\": ").append(log.message).append(",\n\"severity\": ").append(log.severity).append("\n},");
        }
        json = new StringBuilder(json.substring(0, json.length() - 2));
        json.append("}");
        return json.toString();
    }

    public String getRobotPosition() {
        return "{\n\"x\": " + wtd.getPosition().getX() + ",\n\"y\": " + wtd.getPosition().getY() + ",\n \"angle\": " + wtd.getAngle() + "\n}";
    }

    public void start() {
        app.start(port);
    }
}
