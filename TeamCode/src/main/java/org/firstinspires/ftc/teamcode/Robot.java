package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.teamcode.Config.HOOD_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.Config.TURRET_INITIAL_POSITION;

public class Robot {
    DcMotor frontLeftDrive , backLeftDrive, frontRightDrive, backRightDrive,
            intake, shooter, transfer;
    Servo hood, turret;

    private IMU imu         = null;      // Control/Expansion Hub IMU

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
    DcMotor dcMotors[];

    public Robot(HardwareMap hardwareMap) {
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

        dcMotors = new DcMotor[] {
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, intake, shooter, transfer
        };
        resetDCMotors();

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(HOOD_INITIAL_POSITION);

        turret = hardwareMap.get(Servo.class, "turret");
        turret.setPosition(TURRET_INITIAL_POSITION);

        //The next two lines define Hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private void resetDCMotors() {
        for (DcMotor motor: dcMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
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
    public void setTransferPosition(int position, Autonomous autonomous) {
        setMotorPosition(position, transfer, autonomous);
    }
    public void turnIntakeOn() {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(1);
    }
    public void turnIntakeOff() {
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setPower(0);
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
}
