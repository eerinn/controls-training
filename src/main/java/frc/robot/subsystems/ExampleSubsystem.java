// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ExampleSubsystem extends SubsystemBase {
  private final WPI_TalonSRX motor;
  private final WPI_TalonSRX motor2;
  private final WPI_TalonSRX flywheel;
  private final AnalogPotentiometer staging3;
  private final AnalogPotentiometer entrance1;
  private final ColorSensorV3 storage2;
  private final XboxController controller;
  Timer timer;
  

  
  /** Creates a new ExampleSubsystem. */

  public ExampleSubsystem() {
   motor = new WPI_TalonSRX(13);
   motor2 = new WPI_TalonSRX(14);
   flywheel = new WPI_TalonSRX(15);
   staging3 = new AnalogPotentiometer(1);
   entrance1 = new AnalogPotentiometer(0);
   storage2 = new ColorSensorV3(Port.kMXP);  
   controller = new XboxController(0);
   timer = new Timer();
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     
    boolean ballhaspassedsecond;
    boolean threeready;
    boolean shootready;
    boolean lastready;



     // If first sensor has a ball run bottom motor
    if (entrance1.get()>=0.15){
      motor.set(-0.2);
      lastready = false;
    }

    //once ball is past first sensor and is by the second one, continue running bottom motor
    else if (entrance1.get()<=0.15 && storage2.getProximity()>=1600){
      motor.set(-0.2);
    }

    //once the ball is past the second sensor stop the bottom motor and run the top motor
    //only happens if there isn't a ball by the third sensor
    else if (storage2.getProximity()<1600 && staging3.get()<=0.35){
      motor.stopMotor();
      motor2.set(-0.2);
      ballhaspassedsecond = true;
    }

    //once the ball is past second sensor and is by the third sensor, stop the top one
    //bottom one also stops so the second ball will stop once it has passed the second sensor
    else if (storage2.getProximity()<1600 && staging3.get()>0.35){
      motor.stopMotor();
      motor2.stopMotor();
    }

    //stops third ball from going past second sensor
    //if a ball has previously gone past the second sensor, and there is a ball by the second sensor and third sensor, stop the bottom motor.
    //Happens before a button is pressed
    else if (ballhaspassedsecond = true && storage2.getProximity()>=1600 && staging3.get()>0.35 && !controller.getAButtonPressed()){
      motor.stopMotor();
      threeready = true;
      lastready=false;
    }

    //if three balls are in place, and button is pressed, run both motors until no ball is by the second sensor. The top ball will be in position to shoot
    else if (threeready = true && controller.getAButtonPressed() && storage2.getProximity()>=1600){
      motor.set(-0.2);
      motor2.set(-0.2);
      
    }
     else if (threeready = true && storage2.getProximity()<1600){
      motor.stopMotor();
      motor2.stopMotor();
      shootready = true;
      threeready = false;
    }
    
    //if balls are in position to shoot, run the flywheel to shoot top ball
    else if (shootready = true){
      flywheel.set(-0.4);
      timer.start();
      shootready = false;
    }

    else if (timer.hasElapsed(3)){
      flywheel.stopMotor();
      timer.reset();
    }
  }
}   
    
