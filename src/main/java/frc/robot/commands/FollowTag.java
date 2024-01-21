// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import static frc.robot.RobotContainer.*;

// import static java.lang.Math.*;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;



// public class FollowTag extends {
    
//     final PIDController xPID = new PIDController(4, 0, 1.5);
// 	final PIDController yPID = new PIDController(4, 0, 1.5);
// 	final PIDController wPID = new PIDController(4, 0, 4);
    
//     @Override
// 	public void execute() {
// 		if (abs(xPID.getPositionError()) < 0.2) xPID.setI(0.5);
// 		else xPID.setI(0);
// 		if (abs(yPID.getPositionError()) < 0.2) yPID.setI(0.5);
// 		else yPID.setI(0);
// 		if (abs(wPID.getPositionError()) < 20 * Math.PI / 180) wPID.setI(1);
// 		else wPID.setI(0);

// 		double vx = xPID.calculate(odom.now().x, target.x);
// 		double vy = yPID.calculate(odom.now().y, target.y);
// 		double w = wPID.calculate(odom.now().theta, target.theta);

// 		double robot_angle = odom.now().theta;
// 		double vf = cos(robot_angle) * vx + sin(robot_angle) * vy;
// 		double vs = -sin(robot_angle) * vx + cos(robot_angle) * vy;

// 		double max = swerve.renormalize(vx, vy, w, 2);
// 		vf /= max;
// 		vs /= max;
// 		w /= max;

// 		double fvf = cos(robot_angle) * fvx + sin(robot_angle) * fvy;
// 		double fvs = -sin(robot_angle) * fvx + cos(robot_angle) * fvy;

// 		swerve.drive(vf + fvf, vs + fvs, w, 1); // TODO refactor into parameter
// 	}

// }
