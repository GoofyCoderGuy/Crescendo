package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InputManager;
import frc.robot.RobotContainer;


import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Constants.SwerveConstants;
import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
import static frc.robot.RobotContainer.SWERVE;
import static java.lang.Math.*;

public class LimelightDrive extends Command {
    PIDController limelightPID = new PIDController(0.175, 0.0025, 0.016);
    SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
    SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);

    public LimelightDrive() {
        addRequirements(SWERVE);
        limelightPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        limelightPID.reset();
        Logger.recordOutput("Commands/LimeLightDrive", true);
    }

    @Override
    public void execute() {
        double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
        double inputX = inputXYZ[0];
        double inputY = inputXYZ[1];
        double inputMagnitude = Math.hypot(inputX, inputY);
        inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
        double inputDir = Math.atan2(inputY, inputX);
        double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

        double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

        double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);


        double rotationalVelocity = limelightPID.calculate(SWERVE.getPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getLeadingSpeakerAngle().getDegrees());
            
        SWERVE.drive(-yVelocity, -xVelocity, rotationalVelocity);
        
        SmartDashboard.putNumber("Drive error", limelightPID.getPositionError());
        

        LIMELIGHT_INTERFACE.Error = abs(limelightPID.getPositionError()) < 1;

        RobotContainer.logPID("LimeLightPID", limelightPID);
        
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/LimeLightDrive", false);
    }
}
