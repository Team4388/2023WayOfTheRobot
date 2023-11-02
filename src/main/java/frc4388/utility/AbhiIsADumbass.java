// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

/** Exception that occurs if the limelight can't see enough points
 * @author Abhi Sachi
 * @author Aarav Shah
 */
public class AbhiIsADumbass extends Exception {
    /**
     * Creates new AbhiIsADumbass with error text 'null'
     */
    public AbhiIsADumbass() {
        super("Unable to see sufficient vision points (Abhi is a dumbass)");
    }

    /** Creates new AbhiIsADumbass with error text message
     * 
     * @param message Error text message
     */
    public AbhiIsADumbass(String message) {
        super(message);
    }

    /** Creates new AbhiIsADumbass with error text message and detailed stack trace
     * 
     * @param message Error text message
     * @param cause Root cause of error
     */
    public AbhiIsADumbass(String message, Throwable cause) {
        super(message, cause);
    }

    /** Creates new AbhiIsADumbass with error text 'null' and detailed stack trace
     * 
     * @param cause Root cause of error
     */
    public AbhiIsADumbass(Throwable cause) {
        super("Unable to see sufficient vision points (Abhi is a dumbass)", cause);
    }
}
