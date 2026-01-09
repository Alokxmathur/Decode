package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Mid Blue Auto", group="Jr")
//@Disabled
public class MidBlueAuto extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setStartingPosition(Match.StartingPosition.Audience);
        super.setShootingZone(Match.ShootingZone.NearDepot);
        super.runOpMode();
    }
}
