// package frc.robot.Commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;

// import org.littletonrobotics.junction.Logger;

// import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
// import static frc.robot.RobotContainer.SWERVE;
// import static java.lang.Math.abs;


// public class AutoDrive extends Command {
//     PIDController autoPID = new PIDController(0.175, 0.005, 0.015);
//     double rotationalVelocity;
//     public AutoDrive() {
//         addRequirements(SWERVE);
//         autoPID.enableContinuousInput(-180, 180);
//     }

//     @Override
//     public void initialize() {
//         autoPID.reset();
//         Logger.recordOutput("Commands/AutoDrive", true);
//     }

//     @Override
//     public void execute() {
        
//         rotationalVelocity = autoPID.calculate(SWERVE.getPose().getRotation().getDegrees(), LIMELIGHT_INTERFACE.getSpeakerAngle().getDegrees());
//         SWERVE.drive(0, 0, rotationalVelocity);
//         Logger.recordOutput("Drive error", autoPID.getPositionError());
//         RobotContainer.logPID("AmpDrive", autoPID);
//     }

//     @Override
//     public boolean isFinished() {
//         return (abs(autoPID.getPositionError()) < 0.75) && abs(rotationalVelocity) < 0.25;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         SWERVE.drive(0,0,0);
//         Logger.recordOutput("Commands/AutoDrive", false);
//     }
    
// }

