package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Far Blue Auto", group="Jr")
//@Disabled
public class FarBlueAuto extends FarAuto {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearWall);
        super.runOpMode();
    }
}
