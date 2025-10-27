/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name="MAIN_LOWRIDER_TELEOP", group="Linear OpMode")
public class MAIN_LOWRIDER_TELEOP extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    boolean launcherOn = false;
    boolean togglePressed = false;
    double targetRPM = 0.0;
    long lastDpadTime = 0;
    final double MAX_RPM = 5500.0;
    final double RPM_STEP = 500.0;
    final double TICKS_PER_ROTATION = 28.0;

    double verticalPos = 0.5;  // start centered
    double increment = 0.01;   // how much to move each update
    long lastMoveTime = 0;     // time tracker for delay
    long moveDelay = 100;      // delay in ms between moves (adjust as needed)
// === Reset both servos to zero before start ===


    @Override
    public void runOpMode() {
        robot.init();
        robot.vertical.setPosition(0.5);
        robot.vertical2.setPosition(0.5);
        telemetry.addLine("Vertical launcher reset to position 0");
        telemetry.update();


        /*
         Initialize the hardware variables. Note that the strings used here must correspond
         to the names assigned during the robot configuration step on the DS or RC devices.
                frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
                backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
                frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
                backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
         ########################################################################################
         !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
         ########################################################################################
         Most robots need the motors on one side to be reversed to drive forward.
         The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
         If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
         that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
         when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
         Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
         Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        */
/*
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

 */
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            robot.setDrivePower(frontLeftPower, backRightPower, backLeftPower, frontRightPower);

            if (gamepad1.a) {
                robot.intake.setPower(1.0);
            } else if (gamepad1.b) {
                robot.intake.setPower(-1.0);
            } else {
                robot.intake.setPower(0);
            }

            // Toggle launcher ON/OFF with A button
            if (gamepad1.x && !togglePressed) {
                launcherOn = !launcherOn;
                togglePressed = true;
            } else if (!gamepad1.x) {
                togglePressed = false;
            }

            // D-pad up/down to adjust target RPM
            long currentTime = System.currentTimeMillis();
            long debounceDelay = 500; // 250 ms between allowed adjustments
            if (gamepad1.dpad_up && currentTime - lastDpadTime > debounceDelay) {
                targetRPM += RPM_STEP;
                lastDpadTime = currentTime;
            } else if (gamepad1.dpad_down && currentTime - lastDpadTime > debounceDelay) {
                targetRPM -= RPM_STEP;
                lastDpadTime = currentTime;
            }

            // Clamp target RPM within 0 - 5500
            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));

            // Convert RPM → ticks per second using (rpm / 60) * ticks_per_rotation
            double targetTicksPerSecond = (targetRPM / 60.0) * TICKS_PER_ROTATION;

            // Set velocities if launcher is ON, else stop motors
            if (launcherOn) {
                robot.launcher.setVelocity(targetTicksPerSecond);
                robot.launcher2.setVelocity(-targetTicksPerSecond);
            } else {
                robot.launcher.setVelocity(0);
                robot.launcher2.setVelocity(0);
            }

            // --- Get actual RPM from MotorEx (convert ticks/sec to rpm) ---
            double launcherVelocityTicks = robot.launcher.getVelocity();
            double launcher2VelocityTicks = robot.launcher2.getVelocity();

            double launcherRPM = (launcherVelocityTicks / TICKS_PER_ROTATION) * 60.0;
            double launcher2RPM = (launcher2VelocityTicks / TICKS_PER_ROTATION) * 60.0;


// Check if enough time has passed to move again
            if (currentTime - lastMoveTime > moveDelay) {
                if (gamepad1.dpad_right) {
                    verticalPos += increment;  // aim up
                    lastMoveTime = currentTime;
                } else if (gamepad1.dpad_left) {
                    verticalPos -= increment;  // aim down
                    lastMoveTime = currentTime;
                }
            }

// Clip servo positions between 0 and 1
            verticalPos = Math.max(0.0, Math.min(1.0, verticalPos));

// Set both servos (opposite directions)
            robot.vertical.setPosition(verticalPos);
            robot.vertical2.setPosition(1.0 - verticalPos);

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        /*    // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);



         */
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Launcher Active", launcherOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Launcher 1 RPM", "%.1f", launcherRPM);
            telemetry.addData("Launcher 2 RPM", "%.1f", launcher2RPM);
            telemetry.addData("Vertical Servo Pos", "%.2f", verticalPos);
            telemetry.update();
        }
    }
}

