package frc4388.robot.subsystems;

import frc4388.robot.subsystems.Apriltags;

public class Location {
    final Apriltags Apriltag = new Apriltags();

    public boolean isLimelight = false;
    public boolean isApriltag = false;

    //Determines which source to get pos and rot from and also resets
    public void reoderPrio(){
        isLimelight = false; //If limelight gets position and if within a certain range of poles
        isApriltag = Apriltag.isAprilTag();
    }

    public Object[] getPosition() {
        if(isLimelight){
            //Return Limelight Position
        }else if(isApriltag){
            return Apriltag.getApriltagPosition();
        }

        return null;
    }

    public Object[] getRotation() {
        Object[] Rotation = {};

        if(isLimelight){
            //Return Limelight Rotation
        }else if(isApriltag){
            return Apriltag.getApriltagRotation();
        }else{
            //Return odometry Rotation, last resort
        }

        return Rotation;
    }
}
