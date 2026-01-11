package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Depot Blue", group="Jr")
//@Disabled
public class DepotBlueAuto extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setStartingPosition(Match.StartingPosition.Depot);
        super.setShootingZone(Match.ShootingZone.NearDepot);
        super.runOpMode();
    }
}
