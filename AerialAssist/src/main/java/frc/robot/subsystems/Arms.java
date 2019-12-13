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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Arms extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private ArmState armState;
  private ArmPneumaticState pState;
  private double setSpeed;
  private TalonSRX armMotor;
  private DigitalInput limitSwitchIn;
  private DigitalInput limitSwitchOut;
  private DoubleSolenoid solenoidLeft;
  private DoubleSolenoid solenoidRight;
  public Arms() {
    setSpeed = 0;
    armState = ArmState.ARM_STOP;
    armMotor = new TalonSRX(RobotMap.armMotorID);
    limitSwitchIn = new DigitalInput(0);
    limitSwitchOut = new DigitalInput(1);
    solenoidLeft = new DoubleSolenoid(0,1);
    solenoidRight = new DoubleSolenoid(2,3);
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
  public void changeSolenoid(ArmPneumaticState state) {
      this.pState = state;
      if(this.pState == ArmPneumaticState.ARM_CLOSE) {
        solenoidLeft.set(DoubleSolenoid.Value.kReverse);
        solenoidRight.set(DoubleSolenoid.Value.kReverse);
      }
      else if(this.pState == ArmPneumaticState.ARM_OPEN) {
        solenoidLeft.set(DoubleSolenoid.Value.kForward);
        solenoidRight.set(DoubleSolenoid.Value.kForward);
      }
  }

}
