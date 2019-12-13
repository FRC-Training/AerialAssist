/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.TiltState;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Tilt extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double setSpeed;
  private TiltState tState;
  private WPI_TalonSRX tiltMotor;
  private DigitalInput tiltDigitalInput;
  public Tilt() {
    tiltMotor = new WPI_TalonSRX(RobotMap.tiltMotorID);
    tiltDigitalInput = new DigitalInput(1);
    setSpeed = 0;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setMotorSpeed(TiltState state) {
    this.tState = state;
    if(this.tState == TiltState.TILT_DOWN) {
      setSpeed = -0.2;
    }
    else if(this.tState == TiltState.TILT_UP) {
      setSpeed = 0.2;
    }
    if(this.tState == TiltState.TILT_DOWN && tiltDigitalInput.get()) {
      setSpeed = 0;
    }
    tiltMotor.set(ControlMode.PercentOutput, setSpeed);
  }
}
