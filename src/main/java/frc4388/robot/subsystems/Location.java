package frc4388.robot.subsystems;

import frc4388.robot.subsystems.Apriltags.Tag;

public class Location {
    final Apriltags apriltag = new Apriltags();

    private boolean isLimelight = false;
    private boolean isApriltag = false;

    //Determines which source to get pos and rot from and also resets
    private void reoderPrio(){
        isLimelight = false; //If limelight gets position and if within a certain range of poles
        isApriltag = apriltag.isAprilTag();
    }

    public Tag getPosRot() {
        reoderPrio();
        if(isApriltag){
            return apriltag.getTagPosRot();
        } else if (isLimelight) {
            return null;
        }

        return null;
    }
}
