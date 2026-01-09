package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Red Auto", group="Jr")
//@Disabled
public class FarRedAuto extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Red);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearWall);
        super.runOpMode();
    }
}
