// package frc.robot.Commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.InputManager;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.Constants.SwerveConstants;

// import org.littletonrobotics.junction.Logger;

// import static frc.robot.Constants.Constants.SwerveConstants;
// import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
// import static frc.robot.RobotContainer.SWERVE;
// import static java.lang.Math.*;

// public class AmpAim extends Command {
//     PIDController ampAimPID = new PIDController(0.175, 0.005, 0.015);
//     SlewRateLimiter xVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);
//     SlewRateLimiter yVelocityFilter = new SlewRateLimiter(SwerveConstants.slewRateLimit);

//     public AmpAim() {
//         addRequirements(SWERVE);
//         ampAimPID.enableContinuousInput(-180, 180);
//     }

//     @Override
//     public void initialize() {
//         ampAimPID.reset();
//     }

//     @Override
//     public void execute() {
//         double[] inputXYZ = InputManager.getInstance().getDriverXYZAxes();
//         double inputX = inputXYZ[0];
//         double inputY = inputXYZ[1];
//         double inputMagnitude = Math.hypot(inputX, inputY);
//         inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
//         double inputDir = Math.atan2(inputY, inputX);
//         double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

//         double xVelocity = xVelocityFilter.calculate(cos(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);

//         double yVelocity = yVelocityFilter.calculate(sin(inputDir) * inputMagnitude * SwerveConstants.maxSpeed * forwardDirectionSign * SWERVE.speedMultiplier);


//         double rotationalVelocity = ampAimPID.calculate(SWERVE.getPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getLeadingAmpAngle().getDegrees());
//         SWERVE.drive(-yVelocity, -xVelocity, rotationalVelocity);
//         SmartDashboard.putNumber("Drive error", ampAimPID.getPositionError());

//         RobotContainer.logPID("ampAimPID", ampAimPID);
//     }

// }
