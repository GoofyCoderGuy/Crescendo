package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.LIMELIGHT_INTERFACE;


import org.littletonrobotics.junction.Logger;

import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.PivotConstants;


public class PivotSubsystem extends SubsystemBase {
    
    //makes motors
    public final CANSparkFlex pivotMotor1 = new CANSparkFlex(PivotConstants.pivotMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkFlex pivotMotor2 = new CANSparkFlex(PivotConstants.pivotMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

    //gets encoders
    public final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();


    //makes PID for motors
    private final SparkPIDController pivotPIDController = pivotMotor1.getPIDController();

    //motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();


    private double currentPosition = 0;

    private InterpolatingTreeMap<Double,Double> pivotInterp;
    public PivotSubsystem(){

        //sets PID values of both controllers
        pivotPIDController.setP(2);
        pivotPIDController.setI(0);
        pivotPIDController.setD(0);
        pivotPIDController.setOutputRange(-1,1.45);
        pivotPIDController.setFF(0.00015);


        pivotMotor2.follow(pivotMotor1,true);
        //makes encoder account for gear box/Chain
        pivotEncoder.setPositionConversionFactor(1.0/15);
        pivotInterp = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
        pivotInterp.put(0.0,0.0);
        pivotInterp.put(0.24,0.4);
        pivotInterp.put(0.48,1.0);
        pivotInterp.put(1.3,2.8);
;
    }

    public Command raisePivot(){
        return run(
                this::movePivotUp);
    }
    
    public Command lowerPivot(){
        return run(
                this::movePivotDown);
    }

    public Command autoAutoAim(){
        return this.runOnce(this::autoAim);
    }
    public Command movePivotToIntake(){
        return this.runOnce(this::intake);
    }

    public Command movePivotToSubWooferAuto(){
        return this.runOnce(this::SubWoofer);
    }

    public Command movePivotToSubWoofer(){
        return startEnd(
            this::SubWoofer,
            this::intake
        );
    }   
    
    

    public Command BSsetPoint(){
        return this.startEnd(this::BSvoid, this::intake);
    }
    public Command BSsetPoint2(){
        return this.startEnd(this::BSvoid2, this::intake);
    }

    public Command BSsetPoint3(){
        return this.runOnce(this::BSvoid3);
    }

    public Command BSsetPoint4(){
        return this.runOnce(this::BSvoid4);
    }

    public Command printPivotAngle(){
        return new InstantCommand(() -> System.out.println("Current Pivot Position: " + currentPosition + " Distance to Speaker: " + LIMELIGHT_INTERFACE.getSpeakerDistance()));
    }
    //moves pivot up
    private void movePivotUp(){
        currentPosition += 0.005;
    }

    //moves pivot
    private void movePivotDown(){
        currentPosition -= 0.005;
    }

    //uses SparkMax PID to set the motors to a position
    public void intake(){

        pivotPIDController.setP(2);
        pivotPIDController.setI(0.0);
        pivotPIDController.setD(0.0);
        currentPosition = 0.24;
    }

    //uses SparkMax PID to set the motors to a position
    public void SubWoofer(){
        pivotPIDController.setP(0.4);
        pivotPIDController.setI(0.0);
        pivotPIDController.setD(0.0);
        currentPosition = 0;
        
    }

    //uses SparkMax PID to set the motors to a position
    public void AmpPreset(){
        pivotPIDController.setP(1.5);
        pivotPIDController.setI(0.0);
        pivotPIDController.setD(0.0);
        currentPosition = 1.3;
        
    }

    public void passNotePreset(){
        pivotPIDController.setP(2);
        pivotPIDController.setI(0.0);
        pivotPIDController.setD(0.0);
        currentPosition = 0.48;
    }

    public void setCurrentPosition(double SetPoint){
        pivotPIDController.setP(3);
        pivotPIDController.setI(0.0005);
        pivotPIDController.setD(0.0);
        Logger.recordOutput("Pivot/AutoAim/SetPoint", SetPoint);
        currentPosition = MathUtil.clamp(SetPoint, 0, 1.3);
    }

    public void BSvoid(){
        setCurrentPosition(0.316);
    }

    public void BSvoid2(){
        setCurrentPosition(0.22);
    }

    public void BSvoid3(){
        setCurrentPosition(0.21);
    }

    public void BSvoid4(){
        setCurrentPosition(0.09);
    }

    public void autoAim(){
      
        setCurrentPosition(LIMELIGHT_INTERFACE.getSetPoint());
        
    }

     

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position", pivotEncoder.getPosition());

        double kDt = 0.02;
        motor1Setpoint = pivot1Profile.calculate(kDt,motor1Setpoint,motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt,motor2Setpoint,motor2Goal);
        Logger.recordOutput("PivotPID/setPoint", pivotEncoder.getPosition());
        pivotPIDController.setReference(currentPosition, CANSparkFlex.ControlType.kPosition);
        Logger.recordOutput("Pivot", new Pose3d(new Translation3d(-0.13 -0.25, 0.31 -0.25, 0.41),
                new Rotation3d(0, -pivotInterp.get(pivotEncoder.getPosition()), 0)));
        
        
    }
}
