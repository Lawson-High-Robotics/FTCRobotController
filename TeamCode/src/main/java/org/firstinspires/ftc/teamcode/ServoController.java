package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoController {

    /**
     * Maps degrees in range [-180, 180] to servo position [0.0, 1.0].
     *
     * @param servo   The servo to set.
     * @param degrees The desired angle in degrees (-180 to 180).
     */
    public static void setServoDegrees(Servo servo, int degrees) {
        // Clamp degrees to [-180, 180]
        if (degrees < -180) degrees = -180;
        if (degrees > 180) degrees = 180;

        // Map to [0.0, 1.0]
        double position = (degrees + 180) / 360.0;

        servo.setPosition(position);
    }
}
