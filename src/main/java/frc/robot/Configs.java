package frc.robot;

import frc.robot.Constants;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Configs {
    public static final class KrakenConfigs {
        public static final TalonFXConfiguration KRAKEN_CONFIGURATION = new TalonFXConfiguration();

        static {
            KRAKEN_CONFIGURATION.Slot0
                .withKP(ELEVATOR_KP)
                .withKD(ELEVATOR_KD)
                .withKI(ELEVATOR_KI);

            KRAKEN_CONFIGURATION.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            
            KRAKEN_CONFIGURATION.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

            KRAKEN_CONFIGURATION.Feedback
                .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO / CIRCUMFERENCE);
        }
    }
}
