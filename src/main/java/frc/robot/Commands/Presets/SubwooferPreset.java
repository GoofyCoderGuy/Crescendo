// package frc.robot.Commands.Presets;


// import edu.wpi.first.wpilibj2.command.Command;
// import static frc.robot.RobotContainer.PIVOT;
// import static frc.robot.RobotContainer.SHOOTER;


// public class SubwooferPreset extends Command {

//     public SubwooferPreset(){
//         addRequirements(PIVOT,SHOOTER);
//     }
 
//     @Override
//     public void initialize() {
//        PIVOT.SubWoofer();

//     }
//     @Override
//     public void execute() {
//         SHOOTER.setSpeakerShooterMotorSpeedsSubWoofer();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         PIVOT.intake();
//         SHOOTER.setIdleShooterSpeeds();
//     }   
       

// }