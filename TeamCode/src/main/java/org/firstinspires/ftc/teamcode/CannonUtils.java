package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class CannonUtils {

    // Servo angle limits in degrees
    private static final double MIN_ANGLE = 0.0;   // horizontal
    private static final double MAX_ANGLE = 90.0;  // straight up

    // Cannon parameters
    private static final double GRAVITY = 9.81;   // m/s^2
    private static final double INITIAL_VELOCITY = 5.0; // m/s, adjust based on your cannon

    /**
     * Sets servo to angle needed to hit target at distance (in meters)
     */
    public static void aimCannon(Servo servo, double distanceMeters) {
        // Safety check
        if (distanceMeters <= 0) distanceMeters = 0.01;

        // Compute angle in radians
        double angleRad = 0;
        double term = (GRAVITY * distanceMeters) / (INITIAL_VELOCITY * INITIAL_VELOCITY);

        if (term <= 1.0) {
            angleRad = 0.5 * Math.asin(term); // low trajectory
        } else {
            angleRad = Math.PI / 4; // fallback max angle if distance too far
        }

        // Convert to degrees
        double angleDeg = Math.toDegrees(angleRad);

        // Map to servo position
        double servoPos = (angleDeg - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        servoPos = Math.min(1.0, Math.max(0.0, servoPos)); // clamp 0-1

        // Set servo
        servo.setPosition(servoPos);
    }
}
