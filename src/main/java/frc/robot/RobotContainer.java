// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Commands.Presets.AmpPreset;
import frc.robot.Commands.Presets.PassNotePreset;
import frc.robot.Commands.Presets.SubwooferPreset;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Constants.Constants.FieldConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final SwerveSubsystem SWERVE = TunerConstants.DriveTrain;
    public static final Pigeon PIGEON = new Pigeon();

    public static final PivotSubsystem PIVOT = new PivotSubsystem();
    public static final IntakeSubsystem INTAKE = new IntakeSubsystem();
    public static final ShooterSubsystem SHOOTER = new ShooterSubsystem();
    public static final LimelightInterface LIMELIGHT_INTERFACE = new LimelightInterface();
    public static final LEDSubsystem LED_SUBSYSTEM = new LEDSubsystem(INTAKE, SHOOTER);
    public static final HangerSubsystem HANGER = new HangerSubsystem();
    public static final AmpSubsystem AMP = new AmpSubsystem();

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        NamedCommands.registerCommand("shootNote", SHOOTER.autoShootNoteToSpeaker(AMP));
        NamedCommands.registerCommand("shootLimelight", SHOOTER.autoShootNoteLimelight(AMP));
        NamedCommands.registerCommand("intakeNote", INTAKE.autoIntakeNote());
        NamedCommands.registerCommand("revShooter", SHOOTER.revShooter());
        NamedCommands.registerCommand("outtakeNote", INTAKE.outtakeNote());
        NamedCommands.registerCommand("stopSpeakerShooter", SHOOTER.stopShooter());
        NamedCommands.registerCommand("intake", new Handoff(INTAKE, AMP).until(SHOOTER::getShooterSensor));
		NamedCommands.registerCommand("intakeTimeout", new Handoff(INTAKE, AMP).until(SHOOTER::getShooterSensor).withTimeout(2).until(() -> INTAKE.handoff));
        NamedCommands.registerCommand("stopIntake", INTAKE.stopIntake());
        NamedCommands.registerCommand("ampShoot", AMP.shootNoteToAmp());
        NamedCommands.registerCommand("pivotToIntake", PIVOT.movePivotToIntake());
        NamedCommands.registerCommand("backwardShooter", SHOOTER.autoBackwardShooter());
        NamedCommands.registerCommand("autoAim", PIVOT.autoAutoAim());
        NamedCommands.registerCommand("pivotToSubWoofer", PIVOT.movePivotToSubWooferAuto());
        NamedCommands.registerCommand("LimelightDrive", new AutoDrive());
        NamedCommands.registerCommand("Upadate", new InstantCommand(SWERVE::updatePose));
        NamedCommands.registerCommand("UpadateLots", SWERVE.upadate());
        NamedCommands.registerCommand("21", PIVOT.BSsetPoint3());
        NamedCommands.registerCommand("08", PIVOT.BSsetPoint4());


        SWERVE.setDefaultCommand(new DefaultDrive());
        configureBindings();

        SWERVE.setUpPathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
        SmartDashboard.putString("AUTO NAMES", "Name:Four Peice podium: function: four note speaker\n Name:Centerline 4 Function: CenterLine auto\n Name:Shoot Source Side Function: shoot source side no move\n Name: shoot amp side");
    }

    public static boolean IsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Driver Bindings
        InputManager.getInstance().getDriverButton(InputManager.Button.LB_Button5).whileTrue(INTAKE.outtakeNote());
        InputManager.getInstance().getDriverButton(InputManager.Button.RB_Button6).whileTrue(new Handoff(INTAKE, AMP).until(SHOOTER::getShooterSensor).andThen(SHOOTER::setIdleShooterSpeeds));
        InputManager.getInstance().getDriverButton(InputManager.Button.X_Button3).onTrue(new InstantCommand(PIGEON::zeroYaw));
        new Trigger(() -> InputManager.getInstance().getDriverAxis(2) > 0.5).onTrue(SWERVE.slowModeOn()).onFalse(SWERVE.slowModeOff());
        InputManager.getInstance().getDriverButton(InputManager.Button.A_Button1).onTrue(PIVOT.printPivotAngle());
        InputManager.getInstance().getDriverPOV(0).whileTrue(new ConditionalCommand(HANGER.hangerUPTest(), HANGER.hangerUP(),DriverStation::isTest));
        InputManager.getInstance().getDriverPOV(180).whileTrue(new ConditionalCommand(HANGER.hangerDOWNTest(), HANGER.hangerDOWN(),DriverStation::isTest));
        new Trigger(() -> InputManager.getInstance().getDriverAxis(3) > 0.5).whileTrue(new LimelightDrive().alongWith(new AutoAim()));
		InputManager.getInstance().getDriverButton(InputManager.Button.B_Button2).onTrue(new InstantCommand(HANGER::panic));
        InputManager.getInstance().getDriverButton(InputManager.Button.Y_Button4).whileTrue(new AutoDrive());
        InputManager.getInstance().getDriverPOV(270).whileTrue(new LimelightDrive().alongWith(PIVOT.BSsetPoint2()));
        InputManager.getInstance().getDriverPOV(90).whileTrue(PIVOT.BSsetPoint());


        // Operator Bindings
        InputManager.getInstance().getOperatorButton(InputManager.Button.RB_Button6).whileTrue(AMP.shootNoteToAmp());
        InputManager.getInstance().getOperatorButton(InputManager.Button.LB_Button5).whileTrue(SHOOTER.shootNoteToSpeaker());
        InputManager.getInstance().getOperatorButton(InputManager.Button.B_Button2).whileTrue(new LimelightDrive().alongWith(new AutoAim()));
        InputManager.getInstance().getOperatorPOV(270).whileTrue(AMP.ampIntake());
        InputManager.getInstance().getOperatorPOV(90).whileTrue(SHOOTER.shooterBackward());
        InputManager.getInstance().getOperatorPOV(0).whileTrue(PIVOT.raisePivot());
        InputManager.getInstance().getOperatorPOV(180).whileTrue(PIVOT.lowerPivot());
        new Trigger(() -> InputManager.getInstance().getOperatorAxis(2) > 0.5).whileTrue(new AmpAim().alongWith(new PassNotePreset()));
        //new Trigger(() -> InputManager.getInstance().getOperatorAxis(3) > 0.5).whileTrue(new AmpAim().alongWith(PIVOT.movePivotToSubWoofer()).alongWith(SHOOTER.setPassing()));
        new Trigger(() -> InputManager.getInstance().getOperatorAxis(3) > 0.5).whileTrue(new InstantCommand(PIVOT::SubWoofer)); 
        // Operator Presets
        InputManager.getInstance().getOperatorButton(InputManager.Button.Y_Button4).whileTrue(new AmpPreset());
        InputManager.getInstance().getOperatorButton(InputManager.Button.A_Button1).whileTrue(new SubwooferPreset());
        InputManager.getInstance().getOperatorButton(InputManager.Button.X_Button3).whileTrue(new PassNotePreset());
       


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void disableRobot() {
        PIVOT.SubWoofer();
        INTAKE.stopIntake();
        SHOOTER.stopSpeakerShooterMotors();
        AMP.stopAmp();
    }

    public void enableRobot() {
        PIVOT.intake();
        INTAKE.stopIntake();
        SHOOTER.setIdleShooterSpeeds();
        AMP.stopAmp();
        FieldConstants.WING.updateToAlliance();
    }

    public static void logPID(String name, PIDController pid){
        Logger.recordOutput(name+"/error", pid.getPositionError());
        Logger.recordOutput(name+"/setPoint", pid.getSetpoint());
        
    }
}
