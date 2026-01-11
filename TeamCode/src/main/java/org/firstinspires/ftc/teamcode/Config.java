package org.firstinspires.ftc.teamcode;

public class Config {
    public static final double SERVO_INCREMENT = .001f;
    public static final double TURRET_INITIAL_POSITION = .48f;
    public static final double TURRET_FAR_RED_POSITION = .73f;
    public static final double TURRET_FAR_BLUE_POSITION = .17f;
    public static final double HOOD_INITIAL_POSITION = 0.24f;

    //Tile width in inches
    public static final double TILE_WIDTH = 24.0;

    public static final ShootingConfiguration redMidShootingConfiguration =
            new ShootingConfiguration(-81, -45, .5, 1.0, 0.5);
    public static final ShootingConfiguration redFarShootingConfiguration =
            new ShootingConfiguration(-.5*TILE_WIDTH, 0, .73, 0.0, 1.0);
    public static final ShootingConfiguration blueMidShootingConfiguration =
            new ShootingConfiguration(-81, 45, .5, 1.0, 0.5);
    public static final ShootingConfiguration blueFarShootingConfiguration =
            new ShootingConfiguration(-0.5*TILE_WIDTH, 0, .17, 0.0, 1.0);
    public static final ShootingConfiguration redDepotShootingConfiguration =
            new ShootingConfiguration(2.0*TILE_WIDTH, 0, .5, 1.0, 0.5);
    public static final ShootingConfiguration blueDepotShootingConfiguration =
            new ShootingConfiguration(2.0*TILE_WIDTH, 0, .5, 1.0, 0.5);
    public static final LeaveConfiguration redFarLeaveConfiguration = new LeaveConfiguration(0, -48);
    public static final LeaveConfiguration redMidLeaveConfiguration = new LeaveConfiguration(0, 48);
    public static final LeaveConfiguration redDepotLeaveConfiguration = new LeaveConfiguration(45, 48);
    public static final LeaveConfiguration blueFarLeaveConfiguration = new LeaveConfiguration(0, -48);
    public static final LeaveConfiguration blueMidLeaveConfiguration = new LeaveConfiguration(0, 48);
    public static final LeaveConfiguration blueDepotLeaveConfiguration = new LeaveConfiguration(-45, 48);

}
