package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ServoUtilities.calculateServoDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestMode extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int position = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.addData("Position", position);
            telemetry.update();

            if(gamepad1.dpad_left && position <= 180){
               position += 1;
            } else if (gamepad1.dpad_right && position >= -180) {
                position -= 1;
            }

            servo.setPosition(calculateServoDegrees(position));
        }
    }
}