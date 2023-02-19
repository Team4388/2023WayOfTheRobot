package frc4388.robot.subsystems;

//import edu.wpi.first.apriltag.AprilTag;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Apriltags {
    public static class Tag {
        boolean visible = true;
        double x, y, z = 0;
    }
    
    public Object[] getApriltagPosition() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");
        final var tag = new Tag();
        // integrate the tag system
        
        // return new Object[] {true, 
        //     tagTable.getEntry("TagPosX"),
        //     tagTable.getEntry("TagPosY"),
        //     tagTable.getEntry("TagPosZ")
        // };

        return new Object[] {true, 
            1,
            2,
            3
        };
    }

    public Object[] getApriltagRotation() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");

        return new Object[] {true, 
            tagTable.getEntry("TagRotY"),
            tagTable.getEntry("TagRotP"),
            tagTable.getEntry("TagRotR")
        };
    }

    public boolean isAprilTag() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");
        // return tagTable.getEntry("IsTag").getBoolean(false);
        return true;
    }
}
