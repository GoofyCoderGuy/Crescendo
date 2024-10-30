
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.ShooterConstants;

import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;

import org.littletonrobotics.junction.Logger;


public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput speakerSensor = new DigitalInput(4);
    
    private final double motorMaxFreeSpeed = 6784;//RPM
    private boolean shooterIsRunning = false;
    private boolean idleMode = false;
    private PIDController speakerPIDTop = new PIDController((25/motorMaxFreeSpeed),0,0);
    private PIDController speakerPIDBottom = new PIDController((25/motorMaxFreeSpeed),0,0);
    private final CANSparkFlex speakerMotorTop = new CANSparkFlex(ShooterConstants.speakerShooterMotorTopID,
            MotorType.kBrushless);
    private final CANSparkFlex speakerMotorBottom = new CANSparkFlex(ShooterConstants.speakerShooterMotor2ID,
            MotorType.kBrushless);


    RelativeEncoder speakerMotorTopEncoder = speakerMotorTop.getEncoder();
    RelativeEncoder speakerMotorBottomEncoder = speakerMotorBottom.getEncoder();

    public ShooterSubsystem() {
        speakerMotorTop.setSmartCurrentLimit(40);
        speakerMotorBottom.setSmartCurrentLimit(40);
        speakerPIDTop.setTolerance(100);
        speakerPIDBottom.setTolerance(100);

        speakerMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        speakerMotorTop.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
        speakerMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        speakerMotorBottom.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
    }

    public Command shootNoteToSpeaker() {
        return this.startEnd(
                this::setSpeakerShooterMotorSpeeds,
                this::setIdleShooterSpeeds);
    }

    public Command autoShootNoteToSpeaker(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> Logger.recordOutput("Commands/autoShootNoteToSpeak", true)),
                new InstantCommand(this::setSpeakerShooterMotorSpeedsSubWooferAuto),
                new WaitUntilCommand(() -> shooterIsReady()),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.4),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::setIdleShooterSpeeds)).finallyDo(()-> Logger.recordOutput("Commands/autoShootNoteToSpeak", false));
    }

    public Command autoShootNoteLimelight(AmpSubsystem amp) {
        return new SequentialCommandGroup(
                new InstantCommand(this::setSpeakerShooterMotorSpeeds),
                new InstantCommand(amp::setAmpIntakeSpeeds),
                new WaitCommand(0.4),
                new InstantCommand(amp::stopAmpShooterMotorSpeeds),
                new InstantCommand(this::setIdleShooterSpeeds));
    }

    public Command revShooter(){
        return this.runOnce(this::revShooterMotors);
    }


    public Command stopShooter() {
        return this.runOnce(
                this::stopSpeakerShooterMotors);
    }


    public Command shooterBackward(){
        return this.startEnd(
                this::backwardsShooter,
                this::setIdleShooterSpeeds);
    }


    public Command autoBackwardShooter(){
        return new SequentialCommandGroup(
                new InstantCommand(this::backwardsShooter),
                new WaitCommand(3),
                new InstantCommand(this::setIdleShooterSpeeds));
    }

    public Command enableIdleMode(){
        return this.runOnce(this::setIdleShooterSpeeds).onlyWhile(this::getShooterSensor);
    }

    public Command setPassing(){
        return startEnd(
            this::setPassingShooterMotorSpeeds,
            this::setIdleShooterSpeeds);
    }

    public void setIdleShooterSpeeds() {
        speakerPIDTop.setSetpoint(0.5 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.5 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = true;
    }

    public void setSpeakerShooterMotorSpeedsSubWoofer(){
        speakerPIDTop.setSetpoint(0.5 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.5 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void setSpeakerShooterMotorSpeedsSubWooferAuto(){
        speakerPIDTop.setSetpoint(0.35 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.35 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void setSpeakerShooterMotorSpeeds(){
        double shooterSpeeed = 0.6;
        if(LIMELIGHT_INTERFACE.getSpeakerDistance() > 5){
            shooterSpeeed = 0.8;
        }
        speakerPIDTop.setSetpoint(0.8 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(shooterSpeeed * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;

    }
    public void setPassingShooterMotorSpeeds(){
        speakerPIDTop.setSetpoint(0.4 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
        shooterIsRunning = false;
        idleMode = false;
    }


    private void backwardsShooter(){
        speakerPIDTop.setSetpoint(-0.4 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(-0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }

    public void revShooterMotors(){
        speakerPIDTop.setSetpoint(0.6 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }
    public boolean getShooterSensor() {
        return !speakerSensor.get();
    }

    public boolean shooterIsReady(){
        return (speakerPIDTop.atSetpoint() && speakerPIDBottom.atSetpoint() && shooterIsRunning && !idleMode);
    }


    @Override
    public void periodic() {
        if (shooterIsRunning) {
            double outputTop = MathUtil.clamp(speakerPIDTop.calculate(speakerMotorTopEncoder.getVelocity())+speakerPIDTop.getSetpoint()*13/motorMaxFreeSpeed,-10,10);
            double outputBottom = MathUtil.clamp(speakerPIDBottom.calculate(speakerMotorBottomEncoder.getVelocity())+speakerPIDBottom.getSetpoint()*13/motorMaxFreeSpeed,-10,10);

            speakerMotorTop.setVoltage(outputTop);
            speakerMotorBottom.setVoltage(outputBottom);
            // If Issues Persist, Log Data With the Following Code:
           Logger.recordOutput("Shooter/TopMotor/Speed",speakerMotorTopEncoder.getVelocity());
           Logger.recordOutput("Shooter/BottomMotor/Speed",speakerMotorBottomEncoder.getVelocity());
//            Logger.recordOutput("Shooter/TopMotor/PID/Setpoint", speakerPIDTop.getSetpoint());
//            Logger.recordOutput("Shooter/BottomMotor/PID/Setpoint", speakerPIDBottom.getSetpoint());
//            Logger.recordOutput("Shooter/TopMotor/PID/Output", outputTop);
//            Logger.recordOutput("Shooter/BottomMotor/PID/Output", outputBottom);
        }
        if (idleMode && !getShooterSensor()){
            stopSpeakerShooterMotors();
        }
        SmartDashboard.putBoolean("Shooter Sensor", getShooterSensor());

        RobotContainer.logPID("ShooterPIDTop", speakerPIDTop);
        RobotContainer.logPID("ShooterPIDBottom", speakerPIDBottom);
        Logger.recordOutput("Sensors/ShooterSensor", getShooterSensor());
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
