package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Red 6 Spike 1" , group="Curry")
//@Disabled
public class FarRedAuto6Spike1 extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Red);
        super.setSpikeFor3Artifacts(1);
        super.runOpMode();
    }
}