// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import java.util.function.BooleanSupplier;

// Class to get button inputs from Joystick Library/Controllers
public class ControllerUtils {
  private Joystick driver = new Joystick(DriveConstants.kDrveControllerPort);
  private Joystick operator = new Joystick(DriveConstants.kOperControllerPort);

  /**
   * Method of acquiring basic controller buttons
   *
   * @param buttonID Button ID on the controller according to FRC Driver Station
   * @param joystick Controller of which the button is located, use Enum in constants
   */
  public Trigger supplier(int buttonID, DriveConstants.joysticks joystick) {
    if (joystick == DriveConstants.joysticks.DRIVER) {
      BooleanSupplier bsup = () -> driver.getRawButton(buttonID);
      Trigger mybutton = new Trigger(bsup);
      return mybutton;
    } else {
      BooleanSupplier bsup = () -> operator.getRawButton(buttonID);
      Trigger mybutton = new Trigger(bsup);
      return mybutton;
    }
  }

  /**
   * Method of acquiring basic controller button booleans
   *
   * @param buttonID Button ID on the controller according to FRC Driver Station
   * @param joystick Controller of which the button is located, use Enum in constants
   */
  public Boolean Boolsupplier(int buttonID, DriveConstants.joysticks joystick) {
    if (joystick == DriveConstants.joysticks.DRIVER) {
      return driver.getRawButton(buttonID);
    } else {
      return operator.getRawButton(buttonID);
    }
  }

  /**
   * Sub-method of acquiring boolean of controller D-Pad buttons, only for use of POVsupplier method
   *
   * @param degree degree on the D-Pad to be checked (Pulled from FRC Driver Station)
   * @param joystick Controller of which the button is located, use Enum in constants
   */
  public boolean _getPOVbutton(int degree, DriveConstants.joysticks joystick) {
    double point;
    if (joystick == DriveConstants.joysticks.DRIVER) {
      point = driver.getPOV();
      if (point == degree) {
        return true;
      } else {
        return false;
      }
    } else {
      point = operator.getPOV();
      if (point == degree) {
        return true;
      } else {
        return false;
      }
    }
  }

  /**
   * Sub-method of acquiring boolean of controller D-Pad buttons
   *
   * @param angle degree on the D-Pad to be checked (Pulled from FRC Driver Station)
   * @param joystick Controller of which the button is located, use Enum in constants
   */
  public Trigger POVsupplier(int angle, DriveConstants.joysticks joystick) {
    if (joystick == DriveConstants.joysticks.DRIVER) {
      BooleanSupplier bsup = () -> _getPOVbutton(angle, joystick);
      Trigger mybutton = new Trigger(bsup);
      return mybutton;
    } else {
      BooleanSupplier bsup = () -> _getPOVbutton(angle, joystick);
      Trigger mybutton = new Trigger(bsup);
      return mybutton;
    }
  }

  public double inchesToMeters(double inches) {
    return inches * 0.0254;
  }
}
