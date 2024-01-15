package org.firstinspires.ftc.teamcode.pathutils;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class AutonomousWaypoint {

    private static boolean isRed = true;
    private static boolean isBackstage = true;
    /**
     * For points that need maximal precision (ex if you were scoring or crossing through trusses)
     */
    public static double DEFAULT_TOLERANCE = 0.5;
    public static double DEFAULT_HEADING_TOLERANCE = 0.1;
    /**
     * For points that need less precision (ex a point that is a control point but not an end point on the path)
     */
    public static double DEFAULT_IMPRECISE_TOLERANCE = 2;
    public static double DEFAULT_IMPRECISE_HEADING_TOLERANCE = 0.4;
    private static Point GLOBAL_AUDIENCE_OFFSET = new Point(0, 0, 0);
    private static Point GLOBAL_BLUE_OFFSET = new Point(0, 0, 0);

    private double tolerance = DEFAULT_TOLERANCE;
    private double headingTolerance = DEFAULT_HEADING_TOLERANCE;
    private ElapsedTime timeout = new ElapsedTime();

    Point redBackstagePoint;

    /**
     * This variable exists so we can account for different starting positions and still have individual offsets for the audience autos.
     */
    private Point audienceOffset = new Point(0, 0, 0);

    /**
     * This variable exists so we can account for different starting positions and still have individual offsets for the blue autos.
     */
    private Point blueOffset = new Point(0, 0, 0);


    private boolean blueHeadingIsReversed = false;
    private boolean turnToPoint = false;
    private AutonomousWaypoint pointToTurnTo;
    private double rotationOffset = 0;

    /**
     * Offsets for different run positions.
     * Indexed in this order: Red Audience, Blue Backstage, Blue Audience
     */
    ArrayList<Point> offsets = new ArrayList<>(Arrays.asList(new Point(0, 0, 0), new Point(0, 0, 0), new Point(0, 0, 0)));

    public static void configAuto(boolean isRed, boolean isBackstage) {
        AutonomousWaypoint.isRed = isRed;
        AutonomousWaypoint.isBackstage = isBackstage;
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public static void setGlobalAudienceOffset(double x, double y, double rot) {
        AutonomousWaypoint.GLOBAL_AUDIENCE_OFFSET = new Point(x, y, rot);
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public static void setGlobalBlueOffset(double x, double y, double rot) {
        AutonomousWaypoint.GLOBAL_BLUE_OFFSET = new Point(x, y, rot);
    }

    public static void setDefaultTolerance(double tolerance) {
        AutonomousWaypoint.DEFAULT_TOLERANCE = tolerance;
    }

    public static void setDefaultImpreciseTolerance(double tolerance) {
        AutonomousWaypoint.DEFAULT_IMPRECISE_TOLERANCE = tolerance;
    }

    public AutonomousWaypoint setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public AutonomousWaypoint setHeadingTolerance(double tolerance) {
        this.headingTolerance = tolerance;
        return this;
    }

    public AutonomousWaypoint setTolerances(double tolerance, double headingTolerance) {
        this.tolerance = tolerance;
        this.headingTolerance = headingTolerance;
        return this;
    }

    public boolean isAtPoint(Pose2d robotpose) {
        Pose2d endpose = this.getPoint(robotpose).toPose2d();
        endpose = endpose.relativeTo(robotpose);
        /*if (timeout.seconds() > 5) {
            return true;
        }*/
        if (Math.sqrt(Math.pow(endpose.getX(), 2) + Math.pow(endpose.getY(), 2)) < this.tolerance && Math.abs(endpose.getRotation().getRadians()) < this.headingTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public AutonomousWaypoint(double x, double y, double rot) {
        redBackstagePoint = new Point(x, y, rot);
        timeout.reset();
    }

    public AutonomousWaypoint(double x, double y, double rot, boolean imprecise) {
        redBackstagePoint = new Point(x, y, rot);
        if (imprecise) {
            tolerance = DEFAULT_IMPRECISE_TOLERANCE;
            headingTolerance = DEFAULT_IMPRECISE_HEADING_TOLERANCE;
        }
        timeout.reset();
    }

    /**
     * Rotate towards a given point. Please do not make that point reference another point for your own sanity. The rotation of that point does not matter and will not be used.
     */
    public AutonomousWaypoint(double x, double y, AutonomousWaypoint point) {
        turnToPoint = true;
        redBackstagePoint = new Point(x, y, 0);
        pointToTurnTo = point;
        timeout.reset();
    }

    public AutonomousWaypoint(Point p) {
        this(p.x, p.y, p.heading);
    }

    public AutonomousWaypoint(double x, double y, AutonomousWaypoint point, boolean imprecise) {
        turnToPoint = true;
        redBackstagePoint = new Point(x, y, 0);
        pointToTurnTo = point;
        if (imprecise) {
            tolerance = DEFAULT_IMPRECISE_TOLERANCE;
            headingTolerance = DEFAULT_IMPRECISE_HEADING_TOLERANCE;
        }
        timeout.reset();
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public AutonomousWaypoint setAudienceOffset(double x, double y, double rot) {
        audienceOffset = new Point(x, y, rot);
        return this;
    }

    /**
     * Apply for Red Audience coordinates.
     */
    public AutonomousWaypoint setRedAudienceOffset(double x, double y, double rot) {
        offsets.set(0, new Point(x, y, rot));
        return this;
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public AutonomousWaypoint setBlueAudienceOffset(double x, double y, double rot) {
        offsets.set(2, new Point(x, y, rot));
        return this;
    }

    /**
     * Regular offset. Apply from FC coordinates.
     */
    public AutonomousWaypoint setBlueBackstageOffset(double x, double y, double rot) {
        offsets.set(1, new Point(x, y, rot));
        return this;
    }

    public AutonomousWaypoint setBlueHeadingReversed() {
        blueHeadingIsReversed = true;
        return this;
    }

    public AutonomousWaypoint setRotationOffset(double offset) {
        this.rotationOffset = offset;
        return this;
    }


    /**
     * Does not overwrite other configured offsets for blue audience/backstage.
     * Regular offset. Apply from FC coordinates.
     */
    public AutonomousWaypoint setBlueOffset(double x, double y, double rot) {
        blueOffset = new Point(x, y, rot);
        return this;
    }

    public Point getPoint(Pose2d robotPose) {
        if (turnToPoint) {
            Pose2d point = pointToTurnTo.getPoint(robotPose).toPose2d();
            double deltax = point.getX() - robotPose.getX();
            double deltay = point.getY() - robotPose.getY();
            double angle = Math.atan2(deltay, deltax);
            redBackstagePoint.heading = AngleUnit.normalizeRadians(angle + rotationOffset);
            return rawGetPoint();
        } else {
            return rawGetPoint();
        }
    }

    private Point rawGetPoint() {
        Point out = new Point(redBackstagePoint.x, redBackstagePoint.y, redBackstagePoint.heading);
        if (!AutonomousWaypoint.isRed) {
            out = out.reverseY();
            if (blueHeadingIsReversed) {
                out = out.reverseHeading();
            }
            out.offset(blueOffset);
            out.offset(AutonomousWaypoint.GLOBAL_BLUE_OFFSET);
            if (AutonomousWaypoint.isBackstage) {
                out.offset(offsets.get(1));
            } else {
                out.offset(AutonomousWaypoint.GLOBAL_AUDIENCE_OFFSET.reverseY());
                out.offset(audienceOffset.reverseY());
                out.offset(offsets.get(2));
            }
        } else {
            if (!AutonomousWaypoint.isBackstage) {
                out.offset(audienceOffset);
                out.offset(AutonomousWaypoint.GLOBAL_AUDIENCE_OFFSET);
                out.offset(offsets.get(0));
            }
        }
        return out;
    }

    /**
     * For convenience. Use with caution.
     * DO NOT USE THIS METHOD IF YOU ARE TURNING TO A REFERENCE POINT!!!!
     */
    @Deprecated
    // I am very concerned about this function being misused.
    public Point getPoint() {

        if (turnToPoint) {
            Log.println(Log.ERROR, "AutonomousWaypoint", "YOU CALLED GETPOINT BUT YOU DIDNT PASS IN ROBOT POSE AND THIS IS A TURN TO POINT!! THIS WILL NOT WORK!");

            // failsafe: presume we are at the end point and run the method
            return getPoint(new Pose2d(redBackstagePoint.x, redBackstagePoint.y, new Rotation2d(0)));
        } else {
            return rawGetPoint();
        }


    }

    /*public static void main(String[] args) {
        AutonomousWaypoint.configAuto(false, false);
        AutonomousWaypoint.setGlobalBlueOffset(-2, 0, 0);
        Point getpoint = new AutonomousWaypoint(1, -1, 1)
                .setBlueAudienceOffset(-100, -0.5, 0)
                .getPoint();
        System.out.println("(" + getpoint.x + ", " + getpoint.y + ", " + getpoint.heading + ")");
    }*/

    /**
     * Euclidean distance between two AutonomousWaypoints.
     */
    public static double distance(AutonomousWaypoint waypoint1, AutonomousWaypoint waypoint2) {
        // since the rotation doesn't actually matter just pass in blank pose2ds so we dont throw errors
        return Point.distance(waypoint1.getPoint(new Pose2d()), waypoint2.getPoint(new Pose2d()));
    }

    /**
     * Euclidean distance between the robot and an AutonomousWaypoint.
     */
    public static double distance(Pose2d robotPose, AutonomousWaypoint waypoint) {
        // since the rotation doesn't actually matter just pass in blank pose2ds so we dont throw errors
        return Point.distance(new Point(robotPose.getX(), robotPose.getY(), 0), waypoint.getPoint(new Pose2d()));
    }
}
