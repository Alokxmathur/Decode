package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Far Red Auto", group="Jr")
//@Disabled
public class MidBlueAuto extends FarAuto {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearDepot);
        super.runOpMode();
    }
}
