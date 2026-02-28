package org.firstinspires.ftc.teamcode;

public class Config {
    public static final double SERVO_INCREMENT = .005f;
    public static final double POWER_INCREMENT = .005f;
    public static final double TURRET_MIDDLE_POSITION = .5f;
    public static final double HOOD_INITIAL_POSITION = 0.24f;

    //Tile width in inches
    public static final double TILE_WIDTH = 24.0;

    public static final ShootingConfiguration redMidShootingConfiguration =
            new ShootingConfiguration(-72, -45, Config.TURRET_MIDDLE_POSITION, 0.5, 0.4);
    public static final ShootingConfiguration redFarShootingConfiguration =
            new ShootingConfiguration(-0.25*TILE_WIDTH, 0, .78, 0.1, .65);
    public static final ShootingConfiguration blueMidShootingConfiguration =
            new ShootingConfiguration(-72, 45, Config.TURRET_MIDDLE_POSITION, 0.5, 0.4);;
    public static final ShootingConfiguration blueFarShootingConfiguration =
            new ShootingConfiguration(-0.25*TILE_WIDTH, 0, .22, 0.1, .65);
    public static final ShootingConfiguration depotShootingConfiguration =
            new ShootingConfiguration(1.5*TILE_WIDTH, 0, 0.5, 0.5, .35);

    public static final double AUTO_VELOCITY = .4;

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
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    public static final int SHOOT_WHEN_TWO_BALLS_TRANSFER_POSITION = 100;
    public static final int SHOOT_WHEN_ONE_BALL_TRANSFER_POSITION = 170;

    public static final double ROBOT_LENGTH = 18;

}