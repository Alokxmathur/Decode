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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import androidx.annotation.NonNull;


public class Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot;
    private static double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: Rev HD HEx motor
    static final double     DRIVE_GEAR_REDUCTION    = 15 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 104/25.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.


    private Match.Alliance alliance;
    private Match.StartingPosition startingPosition;

    private Match.ShootingZone shootingZone;

    private double delay = 0;

    public void setAlliance(Match.Alliance alliance) {
        this.alliance = alliance;
    }
    public void setStartingPosition(Match.StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
    }
    public void setShootingZone(Match.ShootingZone shootingZone) {
        this.shootingZone = shootingZone;
    }
    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        robot = new Robot(hardwareMap);
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                delay = Math.max(delay-.01, 0);
            }
            else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                delay = Math.min(delay+.01, 10);
            }
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.addData(">", "Delay (max 10 seconds) = %.2f", delay);
            telemetry.update();
        }

        if (delay > 0) {
            sleep((long)delay*1000);
        }
        //find the appropriate shooting setup
        ShootingConfiguration shootingConfiguration = getShootingConfiguration();
        //get robot to shooting position
        assumeShootingStance(shootingConfiguration);

        //shoot first artifact by turning intake on
        robot.turnIntakeOn();
        //wait for artifact to be shot
        sleep(5000);

        //shoot second artifact by pushing two artifacts with the transfer
        robot.setTransferPosition(120, this);
        //shoot third artifact by pushing one artifact with the transfer
        robot.setTransferPosition(0, this);
        robot.setTransferPosition(600, this);
        sleep(3000);

        //retract transfer
        robot.setTransferPosition(0, this);
        //stop shooter
        robot.getShooter().setPower(0);
        //stop intake
        robot.getIntake().setPower(0);
        //set hood to teleop position
        robot.getHood().setPosition(Config.HOOD_INITIAL_POSITION);

        LeaveConfiguration leaveConfiguration = getLeaveConfiguration();
        assumeLeaveStance(leaveConfiguration);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(20000);  // Pause to display last telemetry message.
    }

    @NonNull
    private ShootingConfiguration getShootingConfiguration() {
        ShootingConfiguration shootingConfiguration = null;
        if (alliance == Match.Alliance.Red) {
            if (startingPosition == Match.StartingPosition.Audience) {
                if (shootingZone == Match.ShootingZone.NearWall) {
                    shootingConfiguration = Config.redFarShootingConfiguration;
                }
                else {
                    shootingConfiguration = Config.redMidShootingConfiguration;
                }
            }
            else {
                shootingConfiguration = Config.redDepotShootingConfiguration;
            }
        }
        else {
            if (startingPosition == Match.StartingPosition.Audience) {
                if (shootingZone == Match.ShootingZone.NearWall) {
                    shootingConfiguration = Config.blueFarShootingConfiguration;
                }
                else {
                    shootingConfiguration = Config.blueMidShootingConfiguration;
                }
            }
            else {
                shootingConfiguration = Config.blueDepotShootingConfiguration;
            }
        }
        return shootingConfiguration;
    }
    @NonNull
    private LeaveConfiguration getLeaveConfiguration() {
        LeaveConfiguration leaveConfiguration = null;
        if (alliance == Match.Alliance.Red) {
            if (startingPosition == Match.StartingPosition.Audience) {
                if (shootingZone == Match.ShootingZone.NearWall) {
                    leaveConfiguration = Config.redFarLeaveConfiguration;
                }
                else {
                    leaveConfiguration = Config.redMidLeaveConfiguration;
                }
            }
            else {
                leaveConfiguration = Config.redDepotLeaveConfiguration;
            }
        }
        else {
            if (startingPosition == Match.StartingPosition.Audience) {
                if (shootingZone == Match.ShootingZone.NearWall) {
                    leaveConfiguration = Config.blueFarLeaveConfiguration;
                }
                else {
                    leaveConfiguration = Config.blueMidLeaveConfiguration;
                }
            }
            else {
                leaveConfiguration = Config.blueDepotLeaveConfiguration;
            }
        }
        return leaveConfiguration;
    }
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the OpMode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            robot.getFrontLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getFrontRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getBackLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getBackRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = rightTarget = moveCounts;
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.getFrontLeftDrive().setTargetPosition(moveCounts);
            robot.getFrontRightDrive().setTargetPosition(moveCounts);
            robot.getBackLeftDrive().setTargetPosition(moveCounts);
            robot.getBackRightDrive().setTargetPosition(moveCounts);

            robot.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.getFrontLeftDrive().isBusy() && robot.getFrontRightDrive().isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            robot.stop();
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {
        robot.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - robot.getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.getFrontRightDrive().setPower(rightSpeed);
        robot.getFrontLeftDrive().setPower(leftSpeed);
        robot.getBackRightDrive().setPower(rightSpeed);
        robot.getBackLeftDrive().setPower(leftSpeed);

    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",
                    leftTarget,  rightTarget);
            telemetry.addData("Actual Pos LF:RF:LB:RB",
                    "%7d->%7d:%7d->%7d:%7d->%7d:%7d->%7d",
                    robot.getFrontLeftDrive().getCurrentPosition(),
                    robot.getFrontLeftDrive().getTargetPosition(),
                    robot.getFrontRightDrive().getCurrentPosition(),
                    robot.getFrontRightDrive().getTargetPosition(),
                    robot.getBackLeftDrive().getCurrentPosition(),
                    robot.getBackLeftDrive().getTargetPosition(),
                    robot.getBackRightDrive().getCurrentPosition(),
                    robot.getBackRightDrive().getTargetPosition()
            );
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, robot.getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds LF : RF : LR : RR", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public void assumeShootingStance(ShootingConfiguration configuration) {
        robot.getShooter().setPower(configuration.getShooterSpeed());
        robot.getHood().setPosition(configuration.getHoodPosition());
        robot.getTurret().setPosition(configuration.getTurretPosition());
        driveStraight(.4, configuration.getInitialMovement(), 0);
        turnToHeading(.4, configuration.getHeading());
    }
    public void assumeLeaveStance(LeaveConfiguration configuration) {
        turnToHeading(.4, configuration.getHeading());
        driveStraight(.4, configuration.getDistance(), configuration.getHeading());
    }
}
