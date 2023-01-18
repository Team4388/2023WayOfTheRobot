package frc4388.utility.controller;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4388.utility.controller.XboxController;

/**
 * Mapping for the Xbox controller triggers to allow triggers to be defined as
 * buttons in {@link frc4388.robot.OI}. Checks to see if the given trigger 
 * exceeds a tolerance defined in {@link XboxController}.
 */
public class XboxTriggerButton extends Trigger {
	public static final int RIGHT_TRIGGER = 0;
	public static final int LEFT_TRIGGER = 1;
	public static final int RIGHT_AXIS_UP_TRIGGER = 2;
	public static final int RIGHT_AXIS_DOWN_TRIGGER = 3;
	public static final int RIGHT_AXIS_RIGHT_TRIGGER = 4;
	public static final int RIGHT_AXIS_LEFT_TRIGGER = 5;
	public static final int LEFT_AXIS_UP_TRIGGER = 6;
	public static final int LEFT_AXIS_DOWN_TRIGGER = 7;
	public static final int LEFT_AXIS_RIGHT_TRIGGER = 8;
	public static final int LEFT_AXIS_LEFT_TRIGGER = 9;

	// private XboxController m_controller;
	// private int m_trigger;

	/**
	 * Creates a Trigger-Button mapped to a specific Xbox controller and trigger
	 */
	public XboxTriggerButton(XboxController controller, int trigger) {
		super(() -> get(trigger, controller));
		// m_controller = controller;
		// m_trigger = trigger;
	}

	public static boolean get(int trigger, XboxController controller) {
		if (trigger == RIGHT_TRIGGER) {
			return controller.getRightTrigger();
		}
		else if (trigger == LEFT_TRIGGER) {
			return controller.getLeftTrigger();
		}
		else if (trigger == RIGHT_AXIS_UP_TRIGGER) {
			return controller.getRightAxisUpTrigger();
		}
		else if (trigger == RIGHT_AXIS_DOWN_TRIGGER) {
			return controller.getRightAxisDownTrigger();
		}
		else if (trigger == RIGHT_AXIS_RIGHT_TRIGGER) {
			return controller.getRightAxisRightTrigger();
		}
		else if (trigger == RIGHT_AXIS_LEFT_TRIGGER) {
			return controller.getRightAxisLeftTrigger();
		}
		else if (trigger == LEFT_AXIS_UP_TRIGGER) {
			return controller.getLeftAxisUpTrigger();
		}
		else if (trigger == LEFT_AXIS_DOWN_TRIGGER) {
			return controller.getLeftAxisDownTrigger();
		}
		else if (trigger == LEFT_AXIS_RIGHT_TRIGGER) {
			return controller.getLeftAxisRightTrigger();
		}
		else if (trigger == LEFT_AXIS_LEFT_TRIGGER) {
			return controller.getLeftAxisLeftTrigger();
		}
		return false;
	}
}
