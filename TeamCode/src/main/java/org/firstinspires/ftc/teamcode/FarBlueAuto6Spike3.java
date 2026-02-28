package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Blue 6 Spike 3" , group="Curry")
//@Disabled
public class FarBlueAuto6Spike3 extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setSpikeFor3Artifacts(3);
        super.runOpMode();
    }
}