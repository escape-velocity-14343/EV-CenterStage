package org.firstinspires.ftc.teamcode.pathutils;

import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;

public class AutonomousWaypoint {

    private static boolean isRed = true;
    private static boolean isBackstage = true;
    /**
     * For points that need maximal precision (ex if you were scoring or crossing through trusses)
     */
    private static double DEFAULT_TOLERANCE = 0.5;
    private static double DEFAULT_HEADING_TOLERANCE = 0.1;
    /**
     * For points that need less precision (ex a point that is a control point but not an end point on the path)
     */
    private static double DEFAULT_IMPRECISE_TOLERANCE = 2;
    private static double DEFAULT_IMPRECISE_HEADING_TOLERANCE = 0.4;
    private static Point GLOBAL_AUDIENCE_OFFSET = new Point(0, 0, 0);
    private static Point GLOBAL_BLUE_OFFSET = new Point(0, 0, 0);

    private double tolerance = DEFAULT_TOLERANCE;
    private double headingTolerance = DEFAULT_HEADING_TOLERANCE;

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
        Pose2d endpose = this.getPoint().toPose2d();
        endpose = endpose.relativeTo(robotpose);
        if (Math.sqrt(Math.pow(endpose.getX(), 2) + Math.pow(endpose.getY(), 2)) < this.tolerance && Math.abs(endpose.getRotation().getRadians()) < this.headingTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public AutonomousWaypoint(double x, double y, double rot) {
        redBackstagePoint = new Point(x, y, rot);
    }

    public AutonomousWaypoint(double x, double y, double rot, boolean imprecise) {
        redBackstagePoint = new Point(x, y, rot);
        if (imprecise) {
            tolerance = DEFAULT_IMPRECISE_TOLERANCE;
        }
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


    /**
     * Does not overwrite other configured offsets for blue audience/backstage.
     * Regular offset. Apply from FC coordinates.
     */
    public AutonomousWaypoint setBlueOffset(double x, double y, double rot) {
        blueOffset = new Point(x, y, rot);
        return this;
    }

    public Point getPoint() {
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

    public static void main(String[] args) {
        AutonomousWaypoint.configAuto(false, false);
        AutonomousWaypoint.setGlobalBlueOffset(-2, 0, 0);
        Point getpoint = new AutonomousWaypoint(1, -1, 1)
                .setBlueAudienceOffset(-100, -0.5, 0)
                .getPoint();
        System.out.println("(" + getpoint.x + ", " + getpoint.y + ", " + getpoint.heading + ")");
    }
}
