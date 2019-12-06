/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.RobotMap.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Arms extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private ArmState armState;
  private double setSpeed;
  private TalonSRX armMotor;
  private DigitalInput limitSwitchIn;
  private DigitalInput limitSwitchOut;
  public Arms() {
    setSpeed = 0;
    armState = ArmState.ARM_STOP;
    armMotor = new TalonSRX(RobotMap.armMotorID);
    limitSwitchIn = new DigitalInput(0);
    limitSwitchOut = new DigitalInput(1);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setMotorSpeed(ArmState armState) { 
    this.armState = armState; 
    if(this.armState == ArmState.ARM_STOP) {
      setSpeed = 0;
    }
    else if(this.armState == ArmState.ARM_BACKWARD) {
      setSpeed = -0.2;
    }
    else if(this.armState == ArmState.ARM_FORWARD) {
      setSpeed = 0.2;
    } 
    checkLimitSwitch();
    armMotor.set(ControlMode.PercentOutput,setSpeed);      
  }
  public void checkLimitSwitch() {
    if(limitSwitchIn.get() && armState == ArmState.ARM_BACKWARD) {
      setSpeed = 0;
    }
    else if(limitSwitchOut.get() && armState == ArmState.ARM_FORWARD) {
      setSpeed = 0;
    }
  }

}
