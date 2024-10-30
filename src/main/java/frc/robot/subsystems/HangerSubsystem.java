package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.IntakeConstants;

public class HangerSubsystem extends SubsystemBase{

        private final TalonFX hangerMotor = new TalonFX(IntakeConstants.hangerID);
        
        private final PIDController hangerPID = new PIDController(.02, 0, 0);
        public HangerSubsystem(){
            hangerPID.setSetpoint(0);
            hangerMotor.setPosition(0);
            
       }
      
   
        public Command hangerUP(){
            return this.runOnce(this::hangerUp);
            
        }

        

        public Command hangerDOWN(){
            return runOnce(
                this::hangerDown);
        }

          public Command hangerUPTest(){
            return this.run(
                this::hangerUpTest );
        }

        //-160 top
        //-293
        //-450

        public Command hangerDOWNTest(){
            return run(
                this::hangerDownTest
              );
        }

        public void hangerUp(){
            hangerPID.setSetpoint(-250);
        }

        public void hangerUpTest(){
            hangerPID.setSetpoint(hangerPID.getSetpoint() + 1);
        }

        public void hangerDownTest(){
            hangerPID.setSetpoint(hangerPID.getSetpoint() - 1);
        }

        public void hangerDown(){
         
            hangerPID.setSetpoint(-490);
        }

        public void stopHanger(){
            hangerMotor.stopMotor();
        }

        public double getHangerPosition(){
            return hangerMotor.getPosition().getValueAsDouble();
        }
        public void panic(){
            hangerPID.setSetpoint(getHangerPosition());
        }

   

        @Override
        public void periodic(){
            
            hangerMotor.set(MathUtil.clamp(hangerPID.calculate(getHangerPosition()), -0.9, 0.9));
            SmartDashboard.putNumber("HangerPID", hangerPID.calculate(getHangerPosition()));
            SmartDashboard.putNumber("Hanger Position", getHangerPosition());

            RobotContainer.logPID("hangerPID", hangerPID);
        }

}
