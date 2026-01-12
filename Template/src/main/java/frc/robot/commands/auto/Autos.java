package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    /**
     * Initializes all autonomous utilities and registers auto routines.
     */
    public Autos() {
        AutoUtils.initAutoUtils();
        registerEventMarkers();
        registerAutos();
    }

    private void registerEventMarkers() {
        AutoUtils.registerEventMarker("score", () -> Commands.print("[Auto] Scoring..."));
        AutoUtils.registerEventMarker("intake", () -> Commands.print("[Auto] Intaking..."));
    }

    private void registerAutos() {
        AutoUtils.trajectory("Sample").register();

        AutoUtils.trajectory("Random")
                .atPose("Score", "score")
                .atPose("Intake", "intake")
                .register();

        AutoUtils.trajectory("Complex")
                .atPose("Pickup", () -> Commands.print("[Auto] At pickup"))
                .atTime(1.5, () -> Commands.print("[Auto] 1.5 seconds in"))
                .resetOdometry(true)
                .holdFinalPose(true)
                .register();
    }
}