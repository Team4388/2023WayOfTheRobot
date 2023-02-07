// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.controller;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class Nunchuks extends GenericHID {

    public enum Button {
        kLeftZ(2);
    
        public final int value;
    
        Button(int value) {
          this.value = value;
        }
    
        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
          var name = this.name().substring(1); // Remove leading `k`
          if (name.endsWith("Bumper")) {
            return name;
          }
          return name + "Button";
        }
      }
    
     /** Represents an axis on an XboxController. */
  public enum Axis {
    kLeftX(0),
    kLeftY(1),
    kRightX(3),
    kRightY(4);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

    // public static final int LEFT_X_AXIS = 0;
	// public static final int LEFT_Y_AXIS = 1;
	// public static final int RIGHT_Y_AXIS = 2;
	// public static final int RIGHT_X_AXIS = 3;

    // public static final int LEFT_Z_BUTTON = 2;

    // private static final double DEADZONE = 0.1;

    // private Joystick stick;

    /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public Nunchuks(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

	// /**
    //  * Add your docs here.
    //  */
	// public Nunchuks(int portNumber){
	// 	stick = new Joystick(portNumber);
	// }

     /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

    /**
   * Read the value of the B button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getLeftZButton() {
    return getRawButton(Button.kLeftZ.value);
  }
}
