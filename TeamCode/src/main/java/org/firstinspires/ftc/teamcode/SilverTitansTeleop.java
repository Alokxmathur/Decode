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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Config.POWER_INCREMENT;
import static org.firstinspires.ftc.teamcode.Config.SERVO_INCREMENT;

@TeleOp(name="Teleop", group="Curry")
///@Disabled
public class SilverTitansTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices.

    @Override
    public void runOpMode() {
        boolean runAutomated = false;
        Robot robot = new Robot(hardwareMap, this);
        robot.freeMotorsForTeleOp();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
            }
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            if (!runAutomated) {
                robot.driveWithPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            }

            //handle control of intake using the a, b and y buttons of gamepad 2
            if (gamepad2.b) {
                robot.setIntakePower(1);
            }
            else if (gamepad2.a) {
                robot.lockIntake(this);
            }
            else if (gamepad2.x) {
                robot.setIntakePower(-1);
            }
            //handle control of shooter motor using up and down buttons of gamepad 2
            if (gamepad2.dpad_up) {
                robot.getShooter().setPower(1.0);
            }
            else if (gamepad2.dpad_down) {
                robot.getShooter().setPower(0);
            }
            else if (gamepad2.dpad_left) {
                robot.getShooter().setPower(0.7);
            }
            else if (gamepad2.dpad_right) {
                robot.getShooter().setPower(0.4);
            }
            else if (gamepad2.left_bumper) {
                robot.getShooter().setPower(robot.getShooter().getPower() - POWER_INCREMENT);
            }
            else if (gamepad2.right_bumper) {
                robot.getShooter().setPower(robot.getShooter().getPower() + POWER_INCREMENT);
            }
            //handle control of transfer using right stick of game pad 2
            robot.getTransfer().setPower(-gamepad2.right_stick_y);

            //handle control of turret using left and right dpad of game pad 1
            if (gamepad1.dpad_left) {
                robot.getTurret().setPosition(robot.getTurret().getPosition() - SERVO_INCREMENT);
            } else if (gamepad1.dpad_right) {
                robot.getTurret().setPosition(robot.getTurret().getPosition() + SERVO_INCREMENT);
            }

            //handle control of hood using dpad up and down of game pad 1
            if (gamepad1.dpad_up) {
                robot.getHood().setPosition(robot.getHood().getPosition() + SERVO_INCREMENT);
            }
            else if (gamepad1.dpad_down) {
                robot.getHood().setPosition(robot.getHood().getPosition()  - SERVO_INCREMENT);
            }

            robot.showTelemetry(telemetry, robot.getHeading());
        }
    }
}