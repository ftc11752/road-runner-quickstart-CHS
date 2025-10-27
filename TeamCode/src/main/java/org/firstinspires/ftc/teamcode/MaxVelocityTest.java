package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; /**
 * This OpMode runs a specified motor at full power and reports its velocity in
 * ticks per second to the Driver Station.
 * <p>
 * This is useful for finding the maximum empirical velocity of a motor, which
 * can be used for tuning velocity PID or for Road Runner constants.
 */
@TeleOp(name="Max Velocity Test", group="Utility")
public class MaxVelocityTest extends LinearOpMode {

    // IMPORTANT: Replace "testMotor" with the actual name of your motor
    // configured in the Hardware Map on the Control Hub.
    private static final String MOTOR_NAME = "testMotor";

    private DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the motor hardware
        try {
            motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        } catch (IllegalArgumentException e) {
            telemetry.addData("Error", "Could not find motor named: " + MOTOR_NAME);
            telemetry.update();
            sleep(3000); // Give the user time to read the error
            return;
        }

        // Set the motor to use RUN_WITHOUT_ENCODER mode. This bypasses the
        // motor's internal PID controller, which is what you want for
        // measuring raw max velocity.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the motor to brake when power is zeroed out.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Set the motor to full power (1.0)
            motor.setPower(1.0);

            // Loop until the driver presses STOP
            while (opModeIsActive()) {
                // Get the current velocity from the motor's encoder.
                // The unit is encoder ticks per second.
                double velocity = motor.getVelocity();

                // Report the motor's current velocity to the Driver Station.
                telemetry.addData("Motor", "Running at full power");
                telemetry.addData("Velocity (ticks/sec)", "%.2f", velocity);
                telemetry.addData("Current Power", "%.2f", motor.getPower());
                telemetry.update();
            }

            // Stop the motor when the OpMode ends
            motor.setPower(0);
        }
    }
}
