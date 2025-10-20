package frc.robot.commands.auto;

import frc.robot.config.AutoConfig;

/**
 * @deprecated Use {@link AutoConfig}. The values remain here temporarily for
 *             compatibility with older code paths that still reference
 *             {@code AutoConstants}.
 */
@Deprecated(forRemoval = true)
public final class AutoConstants {

        private AutoConstants() {
        }

        public static final double kPDrive = AutoConfig.Drive.KP;
        public static final double kDDrive = AutoConfig.Drive.KD;

        public static final double kPTurn = AutoConfig.Turn.KP;
        public static final double kDTurn = AutoConfig.Turn.KD;

}