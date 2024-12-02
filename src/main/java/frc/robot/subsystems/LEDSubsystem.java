// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.led.*;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Constants;

// import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;
// import static frc.robot.RobotContainer.SHOOTER;

// import java.awt.Color;

// public class LEDSubsystem extends SubsystemBase {
//     /** Creates a new LED. */
//     private static int NUM_LEDS = 110;
//     private CANdle candle = new CANdle(Constants.LEDConstants.candleID1);
//     private IntakeSubsystem intake;
//     private ShooterSubsystem shooter;

//     private Animation bufferedAnimation = new RainbowAnimation(0.7, 0.2,NUM_LEDS);

//     public LEDSubsystem(IntakeSubsystem intake, ShooterSubsystem shooter) {
//         this.intake = intake;
//         this.shooter = shooter;
//         CANdleConfiguration candleConfiguration = new CANdleConfiguration();
//         candleConfiguration.statusLedOffWhenActive = true;
//         candleConfiguration.disableWhenLOS = false;
//         candleConfiguration.stripType = LEDStripType.RGB;
//         candleConfiguration.brightnessScalar = 1.0;
//         candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
//         candle.configAllSettings(candleConfiguration, 100);
//         candle.clearAnimation(0);
//     }

//     public void setColor(Color color){
//         bufferedAnimation = new StrobeAnimation(color.getRed(), color.getGreen(), color.getBlue(),0,1,NUM_LEDS);
//     }

//     public void strobeAnimation(Color color){
//         bufferedAnimation = new StrobeAnimation(color.getRed(), color.getGreen(), color.getBlue(),0, 0.1,NUM_LEDS);
//     }

//     public void indicateNote() {
//         if (shooter.getShooterSensor() && intake.getHandoffStatus()){
//             setColor(Color.GREEN);
//         }
//         else if (intake.getHandoffStatus()) {
//             setColor(Color.RED);
//         }
//         if (shooter.getShooterSensor()) {
//             setColor(Color.GREEN);
//         }
//     }

//     public void indicateShooter(){
//         if (shooter.shooterIsReady()){
//             setColor(Color.BLUE);
//         }
//         if(shooter.shooterIsReady() && LIMELIGHT_INTERFACE.Error){
//             setColor(Color.MAGENTA);
//         }
//     }

//     public void updateLEDs() {
//         candle.animate(bufferedAnimation);
//         strobeAnimation(Color.BLACK);
//     }

//     @Override
//     public void periodic() {
//         indicateNote();
//         indicateShooter();
//         updateLEDs();

//         SmartDashboard.putBoolean("has two note",shooter.getShooterSensor() && intake.getHandoffStatus() && ( Math.floor(System.currentTimeMillis()/20)) %2 == 0);
//     }
// }