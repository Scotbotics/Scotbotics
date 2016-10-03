import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.IterativeRobot;


/*
 * Simplest program to drive a robot with mecanum drive using a single Logitech
 * Extreme 3D Pro joystick and 4 drive motors connected as follows:
 *   - Digital Sidecar 1:
 *     - PWM 1 - Connected to front left drive motor
 *     - PWM 2 - Connected to rear left drive motor
 *     - PWM 3 - Connected to front right drive motor
 *     - PWM 4 - Connected to rear right drive motor
 */

public class MecanumDefaultCode extends IterativeRobot {
     //Create a robot drive object using PWMs 1, 2, 3 and 4
     RobotDrive m_robotDrive = new RobotDrive(1, 2, 3, 4);
     //Define joystick being used at USB port 1 on the Driver Station
     Joystick m_driveStick = new Joystick(1);

     public void teleopPeriodic() {
          m_robotDrive.mecanumDrive_Cartesian(m_driveStick.getX(), m_driveStick.getY(), m_driveStick.getTwist(),0);
     }
}
