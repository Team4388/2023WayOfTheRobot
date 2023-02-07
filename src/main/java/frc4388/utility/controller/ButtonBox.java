package frc4388.utility.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBox extends GenericHID {
    public ButtonBox(int port) {
        super(port);
    }

    public enum Button {
        kRightSwitch(1),
        kMiddleSwitch(2),
        kLeftSwitch(3),
        kRightButton(4),
        kLeftButton(5);
    
        @SuppressWarnings("MemberName")
        public final int value;
    
        Button(int value) {
            this.value = value;
        }
    }
}