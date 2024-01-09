package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        MotorOutputConfigs angleMotorOutput = swerveAngleFXConfig.MotorOutput;
        angleMotorOutput.Inverted = Constants.ANGLE_MOTOR_INVERT;
        angleMotorOutput.NeutralMode = Constants.AZIMUTH_NEUTRAL_MODE;
        
        /* Current Limiting */
        CurrentLimitsConfigs angleCurrentLimits = swerveAngleFXConfig.CurrentLimits;
        angleCurrentLimits.SupplyCurrentLimitEnable = Constants.AZIMUTH_ENABLE_CURRENT_LIMIT;
        angleCurrentLimits.SupplyCurrentLimit = Constants.AZIMUTH_CURRENT_LIMIT;
        angleCurrentLimits.SupplyCurrentThreshold = Constants.AZIMUTH_CURRENT_THRESHOLD;
        angleCurrentLimits.SupplyTimeThreshold = Constants.AZIMUTH_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        Slot0Configs angleSlot0 = swerveAngleFXConfig.Slot0;
        angleSlot0.kP = Constants.AZIMUTH_P;
        angleSlot0.kI = Constants.AZIMUTH_I;
        angleSlot0.kD = Constants.AZIMUTH_D;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.DRIVE_MOTOR_INVERT;
        driveMotorOutput.NeutralMode = Constants.DRIVE_NEUTRAL_MODE;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = Constants.DRIVE_ENABLE_CURRENT_LIMIT;
        driveCurrentLimits.SupplyCurrentLimit = Constants.DRIVE_CURRENT_THRESHOLD_TIME;
        driveCurrentLimits.SupplyCurrentThreshold = Constants.DRIVE_CURRENT_THRESHOLD;
        driveCurrentLimits.SupplyTimeThreshold = Constants.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = Constants.DRIVE_P;
        driveSlot0.kI = Constants.DRIVE_I;
        driveSlot0.kD = Constants.DRIVE_D;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.CLOSED_LOOP_RAMP;
        
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.CANCODER_INVERT;
    }
}