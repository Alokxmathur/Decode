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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.Config.AUTO_VELOCITY;
import static org.firstinspires.ftc.teamcode.Config.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.Config.TILE_WIDTH;


public class Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot;
    private final Match match = Match.getNewInstance();

    private double delay = 0;

    private int spikeFor3Artifacts = 0;

    public void setAlliance(Match.Alliance alliance) {
        this.match.setAlliance(alliance);
    }

    public void setSpikeFor3Artifacts(int spike) {
        this.spikeFor3Artifacts = spike;
    }

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        robot = new Robot(hardwareMap, this);
        AprilTagDetection pattern = null;
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
            pattern = robot.getObelisk();
            telemetry.addData(">", "Pattern = %s", pattern == null ? "Not seen" : pattern.metadata.name);
            telemetry.update();
            sleep(10);
        }

        if (delay > 0) {
            sleep((long)delay*1000);
        }
        robot.lockIntake(this);
        //wait for us to see the pattern
        while((pattern = robot.getObelisk()) == null) {
            sleep(10);
        }
        Robot.log("Pattern = " + pattern.metadata.name);
        robot.stopStreaming();
        
        //find the appropriate shooting setup
        ShootingConfiguration firstShootingConfiguration = getFirstShootingConfiguration(pattern);
        //get robot to shooting position
        robot.assumeShootingStance(firstShootingConfiguration, this);
        sleep(500);
        //shoot the three artifacts we started with
        robot.shootThreeArtifacts(this);
        robot.getShooter().setPower(0);

        //retract transfer
        robot.setTransferPosition(0, this);
        //turn turret to goal based on alliance
        robot.getTurret().setPosition(this.match.getAlliance() == Match.Alliance.Red ? 1.0 : 0);

        double spikeIntakeHeading = this.match.getAlliance() == Match.Alliance.Red ? 90 : -90;
        //decide how far to move up and how far to go backwards to intake three artifacts from spikes
        double[] movements = getMovementsForSpike(pattern, firstShootingConfiguration, spikeFor3Artifacts);
        Robot.log(String.format(Locale.getDefault(), 
                "Movements=%.2f,%.2f,%.2f,%.2f", 
                movements[0], movements[1], movements[2], movements[3]));

        //move up to line up with the right spike mark
        robot.driveStraight(AUTO_VELOCITY, movements[0], 0, this);
        //turn to face artifacts on spike marks
        robot.turnToHeading(AUTO_VELOCITY, spikeIntakeHeading, this);
        //run into artifacts to intake them
        robot.driveStraight(AUTO_VELOCITY, movements[1], spikeIntakeHeading, this);
        //move away after collecting artifacts
        robot.driveStraight(AUTO_VELOCITY, movements[2], spikeIntakeHeading, this);
        //turn to face audience
        robot.turnToHeading(AUTO_VELOCITY, 0, this);
        //turn shooter on
        robot.getShooter().setPower(.4);
        //drive up to second shooting position
        robot.driveStraight(AUTO_VELOCITY, movements[3], 0, this);
        //lock intake
        robot.lockIntake(this);

        //wait for shooter to speed up
        sleep(2500);

        robot.shootThreeArtifacts(this);
        //stop shooter
        robot.getShooter().setPower(0);
        robot.lockIntake(this);

        //retract to clear white launch line
        robot.driveStraight(AUTO_VELOCITY, TILE_WIDTH, 0, this);

        robot.setTransferPosition(20, this);
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private static double[] getMovementsForSpike(AprilTagDetection pattern,
                                         ShootingConfiguration firstShootingConfiguration,
                                                 int spikeFor3Artifacts) {
        double movementFirstY = 0, movementFirstX = 0, movementSecondX = 0, movementSecondY = 0;
        if (spikeFor3Artifacts == 1 || (spikeFor3Artifacts==0 && pattern.metadata.name.contains("GPP"))) {
            movementFirstY = -(1.5*TILE_WIDTH - ROBOT_LENGTH/2
                    - Math.abs(firstShootingConfiguration.getInitialMovement()) - 4);
            movementFirstX = TILE_WIDTH*2 + 2;
            movementSecondY = -(TILE_WIDTH*2-9);
        }
        else if (spikeFor3Artifacts == 2 || (spikeFor3Artifacts==0 && pattern.metadata.name.contains("PGP"))) {
            movementFirstY = -(2.5*TILE_WIDTH - ROBOT_LENGTH/2
                    - Math.abs(firstShootingConfiguration.getInitialMovement()) - 4);
            movementFirstX = TILE_WIDTH*2 + 2;
            movementSecondY = -(TILE_WIDTH-9);
        }
        else {
            movementFirstY = 0;
            movementFirstX = 2*TILE_WIDTH - 2;
            movementSecondY = -5;
        }
        movementSecondX = -(movementFirstX-2);
        return new double[] {movementFirstY, movementFirstX, movementSecondX, movementSecondY};
    }

    @NonNull
    private ShootingConfiguration getFirstShootingConfiguration(AprilTagDetection pattern) {
        ShootingConfiguration shootingConfiguration = null;

        if (this.match.getAlliance() == Match.Alliance.Red) {
            if (spikeFor3Artifacts > 0) {
                //return first shooting configuration based on spike selected
                switch (spikeFor3Artifacts) {
                    case 1: return Config.redFarShootingConfiguration;
                    case 2: return Config.redFarShootingConfiguration;
                    case 3: return Config.redMidShootingConfiguration;
                }
            }
            else if (pattern.metadata.name.contains("GPP")) {
                shootingConfiguration = Config.redFarShootingConfiguration;
            }
            else if (pattern.metadata.name.contains("PGP")) {
                shootingConfiguration = Config.redFarShootingConfiguration;
            }
            else {
                shootingConfiguration = Config.redMidShootingConfiguration;
            }
       }
        else {
            if (spikeFor3Artifacts > 0) {
                //return first shooting configuration based on spike selected
                switch (spikeFor3Artifacts) {
                    case 1: return Config.blueFarShootingConfiguration;
                    case 2: return Config.blueFarShootingConfiguration;
                    case 3: return Config.blueMidShootingConfiguration;
                }
            }
            else if (pattern.metadata.name.contains("GPP")) {
                shootingConfiguration = Config.blueFarShootingConfiguration;
            }
            else if (pattern.metadata.name.contains("PGP")) {
                shootingConfiguration = Config.blueFarShootingConfiguration;
            }
            else {
                shootingConfiguration = Config.blueMidShootingConfiguration;
            }
        }
        assert shootingConfiguration != null;
        return shootingConfiguration;
    }
}
