package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Blue 6 Spike 1" , group="Curry")
//@Disabled
public class FarBlueAuto6Spike1 extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Blue);
        super.setSpikeFor3Artifacts(1);
        super.runOpMode();
    }
}