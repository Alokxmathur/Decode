package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Blue 6 Spike 2" , group="Curry")
//@Disabled
public class FarBlueAuto6Spike2 extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setSpikeFor3Artifacts(2);
        super.runOpMode();
    }
}