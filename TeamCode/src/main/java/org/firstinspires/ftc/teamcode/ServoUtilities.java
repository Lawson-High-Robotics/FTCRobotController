package org.firstinspires.ftc.teamcode;

public class ServoUtilities {

    /**
     * Maps degrees in range [-180, 180] to servo position [0.0, 1.0].
     *
     * @param degrees The desired angle in degrees (-180 to 180).
     */
    public static double calculateServoDegrees(int degrees) {
        // Clamp degrees to [-180, 180]
        if (degrees < -180) degrees = -180;
        if (degrees > 180) degrees = 180;

        // Map to [0.0, 1.0]
        return (degrees + 180) / 360.0;
    }
}
