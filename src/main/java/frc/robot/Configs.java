package frc.robot;

import frc.robot.Constants;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Configs {
    public static final class MotorConfigs {
        public static final TalonFXConfiguration KRAKEN_CONFIGURATION = new TalonFXConfiguration();

        public static final SparkMaxConfig SPARK_CONFIGURATION = new SparkMaxConfig();
        public static final SparkMaxConfig ELEV_MASTER_CONFIG = new SparkMaxConfig();

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


            SPARK_CONFIGURATION.follow(10, true);
            ELEV_MASTER_CONFIG.inverted(true);

            ELEV_MASTER_CONFIG.encoder
                .positionConversionFactor(1.0 / ELEVATOR_GEAR_RATIO)
                .velocityConversionFactor(1.0 / ELEVATOR_GEAR_RATIO / 60.0);

            SPARK_CONFIGURATION.encoder
                .positionConversionFactor(1.0 / ELEVATOR_GEAR_RATIO)
                .velocityConversionFactor(1.0 / ELEVATOR_GEAR_RATIO / 60.0);
        }

    }
}
