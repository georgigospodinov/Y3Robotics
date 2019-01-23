package main;

import geometry.IntPoint;

import java.util.LinkedList;

import static main.PotentialFieldsRobot.*;

/**
 * This class represents an arc along which the robot moves.
 */
public class Arc {

    private double turnToDo, turnDone;
    private IntPoint start;
    private double chordLength, radius, trackedTravel;
    private double heading;
    private int xChange, yChange;

    public Arc(IntPoint start, IntPoint end, double headingAtStart) {
        turnToDo = 2 * angleChangeTo(start, end, headingAtStart);
        turnDone = 0;
        this.start = new IntPoint(start.x, start.y);
        chordLength = distance(start, end);
        radius = Math.abs(chordLength / (2 * Math.sin(turnToDo / 2)));
        heading = headingAtStart;
        trackedTravel = 0;
    }

    public int getxChange() {
        return xChange;
    }

    public int getyChange() {
        return yChange;
    }

    /**
     * Tells if movement along this arc has been finished by comparing
     * how much of the arc is 'done' and how much in total there is 'to Do'.
     */
    public boolean isCompleted() {
        if (turnToDo != 0)
            return Math.abs(turnDone) >= Math.abs(turnToDo);

        return trackedTravel >= chordLength;
    }

    /**
     * Gives the direction of this arc.
     * -1 means left
     * 1 means right
     */
    public int getSign() {
        return turnToDo < 0 ? -1 : 1;
    }

    /**
     * Gets the heading that the robot will have at the end of the arc.
     */
    public double getHeadingAtEnd() {
        return normalizeAngle(heading - turnToDo + turnDone);
    }

    /**
     * Given a distance to move, returns the orientation that the robot will have after move along the arc.
     * Also updates inner values.
     */
    public double moveBy(int distance) {
        double turnToExecute = (turnToDo < 0 ? -1 : 1) * distance / (2 * radius);
        if (Math.abs(turnToExecute) > Math.abs(turnToDo - turnDone) / 2)
            turnToExecute = (turnToDo - turnDone) / 2;

        // Distance on arc
        double d = turnToExecute == 0 ? distance : Math.abs(2 * radius * Math.sin(turnToExecute));

        // Orient towards:
        double tempHeading = normalizeAngle(heading - turnToExecute);
        xChange = (int) Math.round(d * Math.cos(tempHeading));
        yChange = (int) Math.round(d * Math.sin(tempHeading));
        trackedTravel += Math.sqrt(xChange * xChange + yChange * yChange);

        heading = normalizeAngle(heading - 2 * turnToExecute);
        turnDone += 2 * turnToExecute;

        return heading;
    }

    /**
     * Returns a sequence of points representing the arc.
     * These are used to display it in the GUI.
     */
    public LinkedList<IntPoint> pointSequence(IntPoint point, int subArcLength) {
        LinkedList<IntPoint> points = new LinkedList<>();
        double turnPerChord = (turnToDo < 0 ? -1 : 1) * subArcLength / (2 * radius);
        int numberOfChords = (int) (turnToDo / (2 * turnPerChord)) + 1;
        double d = turnPerChord == 0 ? subArcLength : Math.abs(2 * radius * Math.sin(turnPerChord));

        double tempHeading = heading;
        IntPoint current = point;
        if (current == null) {
            current = start;
        }
        points.addLast(current);
        for (int i = 0; i < numberOfChords; i++) {
            tempHeading = normalizeAngle(tempHeading - turnPerChord);
            int dx = (int) Math.round(d * Math.cos(tempHeading));
            int dy = (int) Math.round(d * Math.sin(tempHeading));
            IntPoint next = new IntPoint(current.x + dx, current.y + dy);
            points.addLast(next);
            tempHeading = normalizeAngle(tempHeading - turnPerChord);
            current = next;
        }
        return points;
    }

    @Override
    public String toString() {
        return String.format("{turnToDo=%s, turnDone=%s, radius=%.2f, start=%s}",
                Math.toDegrees(turnToDo), Math.toDegrees(turnDone), radius, start);
    }
}
