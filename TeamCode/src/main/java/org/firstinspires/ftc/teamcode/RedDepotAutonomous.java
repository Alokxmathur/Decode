/* Copyright (c) 2022 FIRST. All rights reserved.
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

import static org.firstinspires.ftc.teamcode.Config.TILE_WIDTH;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Depot Red" , group="Curry")
//@Disabled
public class RedDepotAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot;
    private final Match match = Match.getNewInstance();

    private double delay = 0;

    public void setAlliance(Match.Alliance alliance) {
        this.match.setAlliance(alliance);
    }

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        robot = new Robot(hardwareMap, this);
        robot.stopStreaming();
        // Wait for the game to start (Display Gyro value, delay and pattern while waiting)
        while (opModeInInit()) {
            if (isStopRequested()) {
                return;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                delay = Math.max(delay-.01, 0);
            }
            else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                delay = Math.min(delay+.01, 10);
            }
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.addData(">", "Delay (max 10 seconds) = %.2f", delay);
            telemetry.update();
            sleep(10);
        }

        if (delay > 0) {
            sleep((long)delay*1000);
        }
        robot.lockIntake(this);

        //get robot to shooting position
        robot.assumeShootingStance(Config.depotShootingConfiguration, this);
        sleep(500);
        //shoot the three artifacts we started with
        robot.shootThreeArtifacts(this);

        //stop shooter

        robot.getShooter().setPower(0);
        robot.setTransferPosition(20, this);

        robot.turnToHeading(.4, 45, this);
        robot.driveStraight(.4, -TILE_WIDTH, 45, this);
        robot.stop();
    }
}
