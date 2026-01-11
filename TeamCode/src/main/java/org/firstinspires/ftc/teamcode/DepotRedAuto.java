package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Depot Red", group="Jr")
//@Disabled
public class DepotRedAuto extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Red);
        super.setStartingPosition(Match.StartingPosition.Depot);
        super.setShootingZone(Match.ShootingZone.NearDepot);
        super.runOpMode();
    }
}
