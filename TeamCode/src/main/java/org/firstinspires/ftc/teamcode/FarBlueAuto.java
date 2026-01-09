package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Blue Auto", group="Jr")
//@Disabled
public class FarBlueAuto extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearWall);
        super.runOpMode();
    }
}
