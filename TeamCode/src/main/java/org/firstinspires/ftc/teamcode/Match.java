package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

public class Match {
    public static String TEAM = "SilverTitans";
    static Match matchInstance;
    public static Match getNewInstance() {
        if (matchInstance == null) {
            matchInstance = new Match();
        }
        return matchInstance;
    }

    synchronized public static Match getInstance() {
        if (matchInstance == null) {
            return getNewInstance();
        }
        else {
            return matchInstance;
        }
    }

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

    public Match() {};
    public Match(Alliance alliance, StartingPosition startingPosition) {
        this.alliance = alliance;
        this.startingPosition = startingPosition;
    }
    public static void log(String s) {
        RobotLog.a(TEAM + ":" + s);
    }
}
