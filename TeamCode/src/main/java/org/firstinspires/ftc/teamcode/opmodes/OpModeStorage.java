package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;

public class OpModeStorage {
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public static boolean red = true;
}
/*
add this to the end of autos to update their ending pose!
@Override
public void stop() {
    OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    //saves alliance color, true is red alliance, false is blue alliance
    OpModeStorage.red = true;
}
 */
