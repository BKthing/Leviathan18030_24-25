package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.reefsharklibrary.data.AngVelConstraint;
import com.reefsharklibrary.data.ConstraintSet;
import com.reefsharklibrary.data.PIDCoeficients;
import com.reefsharklibrary.data.Pose2d;
import com.reefsharklibrary.data.VelConstraint;

@Config
public class RobotConstants {
    public static PIDCoeficients lateralPID = new PIDCoeficients(.24, 0.08, 0.011, 0, 0.06, .0006);//0.012


    public static PIDCoeficients headingPID = new PIDCoeficients(.17, .07, 0.005);//0.012

    public static Pose2d naturalDecel = new Pose2d(180, 180, Math.toRadians(2000));//new Pose2d(210, 210, Math.toRadians(2000))

    public static double trackWidth = 14;

    public static final VelConstraint velConstraint = new VelConstraint(50, 50, 50);

    public static final AngVelConstraint angVelConstraint = new AngVelConstraint(Math.toRadians(300), Math.toRadians(300), Math.toRadians(300));

    public static final ConstraintSet constraints = new ConstraintSet(lateralPID, headingPID, velConstraint, angVelConstraint, naturalDecel, trackWidth, 1.1, .15);
}
