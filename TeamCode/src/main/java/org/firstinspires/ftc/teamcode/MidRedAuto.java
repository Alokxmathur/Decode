package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Far Red Auto", group="Jr")
//@Disabled
public class MidRedAuto extends FarAuto {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Red);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearDepot);
        super.runOpMode();
    }
}
