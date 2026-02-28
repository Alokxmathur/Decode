package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Far Red 6 Spike 3" , group="Curry")
//@Disabled
public class FarRedAuto6Spike3 extends Autonomous {
    @Override
    public void runOpMode() {
        super.setAlliance(Match.Alliance.Red);
        super.setSpikeFor3Artifacts(3);
        super.runOpMode();
    }
}