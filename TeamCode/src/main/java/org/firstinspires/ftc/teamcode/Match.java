package org.firstinspires.ftc.teamcode;

public class Match {
    public enum Alliance {
        Red, Blue
    }
    public enum StartingPosition {
        Depot, Audience
    }

    public enum ShootingZone {
        NearWall, NearDepot;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    Alliance alliance;
    StartingPosition startingPosition;

    public Match(Alliance alliance, StartingPosition startingPosition) {
        this.alliance = alliance;
        this.startingPosition = startingPosition;
    }
}
