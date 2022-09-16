package org.firstinspires.ftc.teamcode.Util;

@Deprecated
public class JavaUtil {
    /**
     * This method returns the indexed byte of an integer.
     *
     * @param data  specifies the integer value.
     * @param index specifies the byte index.
     * @return indexed byte of the integer.
     */
    public static byte intToByte(int data, int index) {
        return (byte) (data >> (8 * index));
    }

    /**
     * This method combines two bytes into an integer.
     *
     * @param data1 specifies the lowest byte.
     * @param data2 specifies the next lowest byte.
     * @param data3 specifies the next byte.
     * @param data4 specifies the highest byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data1, byte data2, byte data3, byte data4) {
        return (data4 << 24) & 0xff000000
                | (data3 << 16) & 0x00ff0000
                | (data2 << 8) & 0x0000ff00
                | data1 & 0x000000ff;
    }

    /**
     * This method combines two bytes into an integer.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte low, byte high) {
        return bytesToInt(low, high, (byte) 0, (byte) 0);
    }

    /**
     * This method converts a byte into an integer.
     *
     * @param data specifies the byte data.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data) {
        return data;
    }

    /**
     * This method combines two bytes into a short.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted short.
     */
    public static short bytesToShort(byte low, byte high) {
        return (short) bytesToInt(low, high);
    }
}
