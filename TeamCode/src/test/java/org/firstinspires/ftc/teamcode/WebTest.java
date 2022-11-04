package org.firstinspires.ftc.teamcode;

import io.javalin.testtools.JavalinTest;

import org.firstinspires.ftc.teamcode.Subsystems.Web.Web;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.robolectric.RobolectricTestRunner;

import static org.junit.jupiter.api.Assertions.assertEquals;

//@ExtendWith(RobolectricTestRunner.class)
public class WebTest {
    Web init() {
        return new Web(7070);
    }

    @Test
    void testVersioning() {
        assertEquals(1,1);
        Web web = init();
        JavalinTest.test(web.app, (server, client) -> {
            assertEquals(client.get("/").code(), 200);
//            assertEquals(client.get("/").body().string(), "");
        });
    }
}
