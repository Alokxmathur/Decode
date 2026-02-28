package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.operations.OperationThread;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.Config.AUTO_VELOCITY;
import static org.firstinspires.ftc.teamcode.Config.HOOD_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.Config.TURRET_MIDDLE_POSITION;

public class Robot {
    DcMotor frontLeftDrive , backLeftDrive, frontRightDrive, backRightDrive,
            intake, shooter, transfer;
    Servo hood, turret;

    private IMU imu         = null;      // Control/Expansion Hub IMU

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // Rev HD motor
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // Geared down 20:1.
    static final double     WHEEL_DIAMETER_INCHES   = 104.0/25.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getBackLeftDrive() {
        return backLeftDrive;
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getBackRightDrive() {
        return backRightDrive;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getShooter() {
        return shooter;
    }

    public DcMotor getTransfer() {
        return transfer;
    }

    public Servo getHood() {
        return hood;
    }

    public Servo getTurret() {
        return turret;
    }

    public IMU getIMU() {
        return imu;
    }
    DcMotor[] dcMotors;

    OperationThread operationThread;

    //adding OTOS for locaization

    public Robot(HardwareMap hardwareMap, LinearOpMode opMode) {
        operationThread = new OperationThread(this, "Primary", opMode);
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotor.Direction.REVERSE);

        dcMotors = new DcMotor[] {
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, intake, transfer
        };
        resetDCMotors();

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(HOOD_INITIAL_POSITION);

        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(TURRET_MIDDLE_POSITION);

        //The next two lines define Hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        initAprilTagProcessor(hardwareMap);
    }

    private void resetDCMotors() {
        for (DcMotor motor: dcMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //special configuration of the shooter to allow 100% of the speed
        MotorConfigurationType motorConfiguration = this.shooter.getMotorType().clone();
        //allow 100% of speed, default is 85%
        motorConfiguration.setAchieveableMaxRPMFraction(1.0);
        this.shooter.setMotorType(motorConfiguration);
        this.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveWithPowers(
            double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower)
    {
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void setIntakePosition(int position, LinearOpMode opMode) {
        setMotorPosition(position, intake, opMode);
    }
    public void setTransferPosition(int position, LinearOpMode opMode) {
        setMotorPosition(position, transfer, opMode);
    }
    public void turnIntakeOn() {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(1);
    }
    public void turnIntakeOff() {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(0);
    }
    public void lockIntake(LinearOpMode opMode) {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorPosition(0, this.intake, opMode);
    }
    public void reverseIntake() {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(-1);
    }
    private void setMotorPosition(int position, DcMotor motor, LinearOpMode opMode) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        while (opMode.opModeIsActive() && motor.isBusy()) {
            opMode.sleep(100);
        }
    }
    public void freeMotorsForTeleOp() {
        for (DcMotor motor: dcMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setIntakePower(double power) {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(power);
    }
    
    public void showTelemetry(Telemetry telemetry, double desiredHeading) {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", this.getFrontLeftDrive().getPower(), this.getFrontRightDrive().getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", this.getBackLeftDrive().getPower(), this.getBackRightDrive().getPower());
        telemetry.addData("Transfer", "%d->%d@%.2f",
                this.getTransfer().getCurrentPosition(), this.getTransfer().getTargetPosition(), this.getTransfer().getPower());
        telemetry.addData("Intake / Shooter", "%d->%d@%.2f, %d->%d@%.2f",
                this.getIntake().getCurrentPosition(), this.getIntake().getTargetPosition(), this.getIntake().getPower(),
                this.getShooter().getCurrentPosition(), this.getShooter().getTargetPosition(), this.getShooter().getPower());
        telemetry.addData("Hood / Turret", "%4.2f, %4.2f",
                this.getHood().getPosition(), this.getTurret().getPosition());
        telemetry.addData("Heading / Desired ", "%4.2f, %4.2f", this.getHeading(), desiredHeading);
        telemetry.update();
    }
    private void initAprilTagProcessor(HardwareMap hardwareMap) {

        // Create the AprilTag processor the easy way.
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }   // end method initAprilTag()

    public AprilTagDetection getObelisk() {
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                return detection;
            }
        }
        return null;
    }
    public static void log (String message) {
        RobotLog.i("SilverTitans: %s", message);
    }

    public void stopStreaming() {
        this.visionPortal.close();
    }

    public void shootThreeArtifacts(LinearOpMode opMode) {
        //shoot first artifact by turning intake on
        turnIntakeOn();
        //wait for artifact to be shot
        opMode.sleep(2000);

        shootTwoArtifacts(opMode);
    }

    public void shootTwoArtifacts(LinearOpMode opMode) {
        //shoot second artifact by pushing two artifacts with the transfer
        setTransferPosition(Config.SHOOT_WHEN_TWO_BALLS_TRANSFER_POSITION, opMode);
        //shoot third artifact by pushing one artifact with the transfer
        //setTransferPosition(30, this);
        //opMode.sleep(1000);
        setTransferPosition(Config.SHOOT_WHEN_ONE_BALL_TRANSFER_POSITION, opMode);
        //opMode.sleep(500);
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
                                     double heading,
                                    LinearOpMode opMode) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {
            getFrontLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getFrontRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getBackLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getBackRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            getFrontRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            getBackRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * Config.COUNTS_PER_INCH);
            // Set Target FIRST, then turn on RUN_TO_POSITION
            getFrontLeftDrive().setTargetPosition(moveCounts);
            getFrontRightDrive().setTargetPosition(moveCounts);
            getBackLeftDrive().setTargetPosition(moveCounts);
            getBackRightDrive().setTargetPosition(moveCounts);

            getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            getFrontRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            getBackLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            getBackRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (getFrontLeftDrive().isBusy() && getFrontRightDrive().isBusy())) {

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, getHeading(), Config.P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed);

                // Display drive status for the driver.
                showTelemetry(opMode.telemetry, heading);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stop();
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the desiredHeading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param desiredHeading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current desiredHeading.
     */
    public void turnToHeading(double maxTurnSpeed, double desiredHeading, LinearOpMode opMode) {
        getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getFrontRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getBackRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // keep looping while we are still active, and not on desiredHeading.
        while (opMode.opModeIsActive() && (Math.abs(desiredHeading - getHeading()) > Config.HEADING_THRESHOLD)) {
            // Determine required steering to keep on desiredHeading
            double turnSpeed = getSteeringCorrection(desiredHeading, getHeading(), Config.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            Thread.yield();
            // Display drive status for the driver
            showTelemetry(opMode.telemetry, desiredHeading);

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
    public double getSteeringCorrection(double desiredHeading, double currentHeading, double proportionalGain) {
        // Determine the heading current error
        double headingError = desiredHeading - currentHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param driveSpeed forward motor speed
     * @param turnSpeed  clockwise turning motor speed.
     */
    public void moveRobot(double driveSpeed, double turnSpeed) {

        double leftSpeed  = driveSpeed - turnSpeed;
        double rightSpeed = driveSpeed + turnSpeed;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        getFrontRightDrive().setPower(rightSpeed);
        getFrontLeftDrive().setPower(leftSpeed);
        getBackRightDrive().setPower(rightSpeed);
        getBackLeftDrive().setPower(leftSpeed);

    }
    public void assumeShootingStance(ShootingConfiguration configuration, LinearOpMode opMode) {
        getShooter().setPower(configuration.getShooterSpeed());
        getHood().setPosition(configuration.getHoodPosition());
        getTurret().setPosition(configuration.getTurretPosition());
        driveStraight(AUTO_VELOCITY, configuration.getInitialMovement(), getHeading(), opMode);
        turnToHeading(AUTO_VELOCITY, configuration.getHeading(), opMode);
    }

}
