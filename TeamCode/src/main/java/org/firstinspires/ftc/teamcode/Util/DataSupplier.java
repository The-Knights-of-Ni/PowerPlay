package org.firstinspires.ftc.teamcode.Util;

/**
 * This interface provides the method to get data of the specified type. This is to replaced the
 * Supplier interface that Java SDK provides but Android API level 19 does not have.
 */
public interface DataSupplier<T> {
    /**
     * This method returns the data of the designated type.
     *
     * @return data of the designated type.
     */
    T get();
}
