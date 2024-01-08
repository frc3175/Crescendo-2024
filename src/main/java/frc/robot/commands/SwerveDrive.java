package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDrive extends Command {    
    private SwerveDrivetrain m_swerveDrivetrain;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    private BooleanSupplier m_robotCentricSup;
    private SlewRateLimiter m_xAxisLimiter;
    private SlewRateLimiter m_yAxisLimiter;
    private BooleanSupplier m_isEvading;

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier isEvading) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_robotCentricSup = robotCentricSup;
        m_isEvading = isEvading;

        m_xAxisLimiter = new SlewRateLimiter(Constants.Swerve.RATE_LIMITER);
        m_yAxisLimiter = new SlewRateLimiter(Constants.Swerve.RATE_LIMITER);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double xAxis = MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
        double yAxis = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
        double rAxis = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.stickDeadband);

        double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
        double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);
        double rAxisSquared = rAxis * rAxis * Math.signum(rAxis);

        double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
        double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);

        /* Drive */
        m_swerveDrivetrain.drive(
            new Translation2d(xAxisFiltered, yAxisFiltered).times(Constants.Swerve.maxSpeed), 
            rAxisSquared * Constants.Swerve.maxAngularVelocity, 
            !m_robotCentricSup.getAsBoolean(), 
            true,
            m_isEvading.getAsBoolean()
        );
    }
}