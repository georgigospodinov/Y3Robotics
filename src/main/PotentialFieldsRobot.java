package main;

import geometry.IntPoint;
import renderables.*;

import java.awt.*;
import java.awt.geom.Line2D;
import java.io.File;
import java.util.*;
import java.util.List;

public class PotentialFieldsRobot {

    public static final double SPECIAL = 0.5837;
    public static final int MAX_TURN = 12;
    public static final double SAFE_DISTANCE_FACTOR = 1.5;

    private final RenderableImg robotPic; //Renderable image of the robot
    private final RenderablePoint robotPicAlt; //Default image if no picture is provided
    private IntPoint coords; //Position of robot
    private double heading; //Robot's heading in radians
    private final int radius; //Size of the robot (Our robot is a circle)
    private final int sensorRange; //Range of sensors
    private final int stepSize; //How far the robot moves each step, in pixels
    private final int sensorDensity; //Number of 'lines' the robot uses to see
    private IntPoint goal;
    private int goalRadius;
    private final List<Renderable> obstacles; //All of the obstacles on the map
    private List<IntPoint> visibleObstacles;
    private int sampleSize;
    private final int sampleSizeDefault;
    /* Added: */
    private final boolean useUnwind;
    private Arc first, second, third, unwinding;
    private final boolean arc;
    private boolean isWinding, isUnwinding;
    private double windAmount = 0.0;
    private LinkedList<IntPoint> arcPoints = new LinkedList<>();
    private LinkedList<IntPoint> safetyPoints = new LinkedList<>();
    private final LinkedHashMap<Integer, LinkedHashMap<Integer, Integer>> occurrences = new LinkedHashMap<>();
    private final LinkedList<IntPoint> pts = new LinkedList<>();

    /**
     * Normalizes an given angle via rotating it by 2 PI until it falls in the range [0, 2*PI).
     */
    public static double normalizeAngle(double angle) {
        while (angle < 0)
            angle += 2 * Math.PI;

        while (angle >= 2 * Math.PI)
            angle -= 2 * Math.PI;

        return angle;
    }

    /**
     * List the robot moving towards a goal on the screen with List radius, step size, etc.
     *
     * @param imagePath     The on-disk location of the image to use for the robot, or null for default
     * @param start         The coordinates of the starting point
     * @param goal          The coordinates of the goal
     * @param radius        The radius of the robot
     * @param sensorRange   How far the robot can 'see'
     * @param sensorDensity The number of sensor lines the robot can use
     * @param goalRadius    The width of the goal
     * @param obstacles     A list of all the obstacles on the map
     */
    public PotentialFieldsRobot(String imagePath, IntPoint start, IntPoint goal, int radius,
                                int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles, double initialOrientation, boolean arc, boolean useUnwind) {
        if (imagePath == null) {
            robotPic = null;
            robotPicAlt = new RenderablePoint(start.x, start.y);
            robotPicAlt.setProperties(Color.RED, (float) radius * 2);
        }
        else {
            robotPicAlt = null;
            robotPic = new RenderableImg("images" + File.separator + imagePath, start.x, start.y, radius * 2, radius * 2);
        }
        this.coords = new IntPoint(start.x, start.y);
        this.heading = Math.toRadians(initialOrientation);
        this.radius = radius;
        this.sensorRange = sensorRange;
        this.sensorDensity = sensorDensity;
        this.stepSize = 10;
        this.sampleSizeDefault = 2 * radius;
        this.goal = goal;
        this.goalRadius = goalRadius;
        this.obstacles = obstacles;
        this.arc = arc;
        this.useUnwind = useUnwind;
    }

    /**
     * Used to display an arc on the screen.
     */
    public LinkedList<IntPoint> getArcPoints() {
        if (isUnwinding && unwinding != null) return unwinding.pointSequence(coords, stepSize);
        return isWinding ? new LinkedList<>() : arcPoints;
    }

    /**
     * Used to display the points that should be visible before it is safe to leave.
     */
    public LinkedList<IntPoint> getSafetyPoints() {
        return safetyPoints;
    }

    private void countPoint(IntPoint p) {
//        System.out.println("counting p = " + p);
        if (!occurrences.containsKey(p.x)) {
            System.out.println("creating new for " + p.x);
            occurrences.put(p.x, new LinkedHashMap<>());
        }

        LinkedHashMap<Integer, Integer> vals = occurrences.get(p.x);
        Integer v = vals.get(p.y);
        if (v == null) {
            System.out.println("creating " + p.x + "  " + p.y);
            pts.add(new IntPoint(p.x, p.y));
//            System.out.println("pts.size= " + pts.size());
            vals.put(p.y, 1);
        }
        else {
            vals.put(p.y, v+1);
        }
    }

    private int getOccurrences(IntPoint p) {
        System.out.println("px = "+p.x);
        System.out.println("occurrence px = "+occurrences.get(p.x));
        if (!occurrences.containsKey(p.x)) {
//            System.out.println(p.x + " not found");
            return 0;
        }
        LinkedHashMap<Integer, Integer> vals = occurrences.get(p.x);

        Integer v = vals.get(p.y);
        if (v == null) {
//            System.out.println("(" + p.x + "," + p.y + ") not found");
            return 0;
        }

        return v;
    }

    private double distanceToClosestHistory(IntPoint p) {
        double minDist = Double.MAX_VALUE;
        Set<Integer> xs = occurrences.keySet();
        for (int x : xs) {
            LinkedHashMap<Integer, Integer> os = occurrences.get(x);
            Set<Integer> ys = os.keySet();
            for (int y : ys) {
                IntPoint point = new IntPoint(x,y);
                double d = distance(p, point);
                if (d < minDist) minDist = d;
            }
        } //for xs

        return minDist;
    }

    private double getPointValue(IntPoint p) {
        Integer i = getOccurrences(p);
        double nom = Math.pow(10, i);
//        System.out.println("nom = " + nom);
        if (i != 0) {
            return nom;
        }

        double den = distanceToClosestHistory(p);

        System.out.println("nom/den = " + nom / den);
        return nom / den;
    }

    public LinkedList <IntPoint> getPointsIveBeen() {
        return pts;
    }

    /**
     * Given a sine and a cosine of an angle, determine what that angle is.
     */
    public static double asincos(double sinAlpha, double cosAlpha) {
        double angle = Math.asin(sinAlpha);  //returns a value between -pi/2 and pi/2
        if (cosAlpha < 0)  // accounts for sin(x) == sin(180-x)
            angle = Math.PI - angle;
        return normalizeAngle(angle);
    }

    /**
     * Calculates the angle that the given points form with the x-axis.
     * An imaginary horizontal line passes through A and the angle from B to A is returned.
     */
    public static double angleWithXAxis(IntPoint A, IntPoint B) {
        double xdiff = B.x - A.x, ydiff = B.y - A.y;
        double dist = distance(A, B);
        double sinAlpha = ydiff / dist, cosAlpha = xdiff / dist;
        return asincos(sinAlpha, cosAlpha);
    }

    /**
     * Given an origin point, a destination, and an angle at the origin,
     * determine how much the robot needs to turn in order to be facing the destination.
     */
    public static double angleChangeTo(IntPoint origin, IntPoint destination, double angleAtOrigin) {
        double angleToPoint = angleWithXAxis(origin, destination);
        double theta = angleAtOrigin - angleToPoint;

        if (theta > Math.PI)
            theta = theta - (2 * Math.PI);
        else if (theta < -Math.PI)
            theta = theta + 2 * Math.PI;

        return theta;
    }

    /**
     * Move the robot 1 step towards the goal.
     * If the robot is supposed to move in an arc, the arcMove method is used.
     * Otherwise the forwardsMove method is called.
     *
     * @return True if the move is successful, false if there are no viable moves.
     **/
    public boolean move() {
        return arc ? arcMove() : forwardsMove();
    }

    /**
     * This is the enhanced version of the provided move method.
     * It's operation differs in no way from the original.
     * It does the same job but more efficiently.
     */
    private boolean forwardsMove() {
        //Pick a sample point to move towards
//        IntPoint moveTo = evalPoints(getSamplePoints(), this.goal);
//        if (moveTo == null) {
//            System.out.println("moveTo was null");
//            return false;
//        }

        //Find the best move point using current sample as goal
//        IntPoint makeMove = evalPoints(getMovablePoints(), moveTo);
        IntPoint makeMove = evalPoints(getSensorablePoints(), goal);
        if (makeMove == null) {
            System.out.println("makeMove was null");
            return false;
        }

        double newHeading = calculateHeading(makeMove);
        moveTowards(newHeading); //Make the move
        return true;
    }

    /**
     * Finds the best point to form an arc to.
     */
    private IntPoint findBestPoint() {
        IntPoint best = evalPoints(getSensorablePoints(), goal);
        if (best == null) return null;

        // when we are close to the goal
        if (distance(coords, goal) <= distance(coords, best))
            best = goal;

        return best;
    }

    /**
     * Determines if a point is a safe distance away from any visible obstacle.
     */
    private boolean isPointSafe(IntPoint p) {
        for (IntPoint obstacle : visibleObstacles) {
            if (distance(p, obstacle) <= radius * SAFE_DISTANCE_FACTOR)
                return false;
        }

        return true;
    }

    /**
     * Determines if a point is a safe destination.
     * That is the line to that point does not intersect an obstacle and
     * that point is safe (see method above).
     */
    private boolean isDestinationSafe(IntPoint p) {
        Line2D.Double lineToFront = new Line2D.Double(coords.x, coords.y, p.x, p.y);
        return intersects(lineToFront) == null && isPointSafe(p);
    }

    /**
     * Determines if a given arc collides with an obstacle.
     */
    private boolean checkCollision(Arc arc) {
        LinkedList<IntPoint> points = arc.pointSequence(coords, stepSize);
        for (IntPoint p : points)
            if (!isPointSafe(p)) return true;

        return false;
    }

    /**
     * Re-constructs the arcs as described in the 'arcs and heading constrcution' document.
     * The given point is used as the end point for the first arc.
     */
    private Arc reconstructArcs(IntPoint sample) {
        first = new Arc(coords, sample, heading);
        planNextTwoArcs(sample);
        arcPoints = new LinkedList<>();
        arcPoints.addAll(first.pointSequence(null, stepSize));
        arcPoints.addAll(second.pointSequence(null, stepSize));
        arcPoints.addAll(third.pointSequence(null, stepSize));
        return first;
    }

    /**
     * Calls {@link PotentialFieldsRobot#findBestPoint()}
     * and then {@link PotentialFieldsRobot#reconstructArcs(IntPoint)} with it.
     */
    private Arc reconstructArcs() {
        IntPoint best = findBestPoint();
        if (best == null) return null;
        return reconstructArcs(best);
    }

    /**
     * Returns the arc that the robot is currently moveing on.
     */
    private Arc getCurrentArc() {
        Arc current;
        if (first == null) current = null;
        else if (!first.isCompleted()) current = first;
        else if (!second.isCompleted()) current = second;
        else if (!third.isCompleted()) current = third;
        else current = null;
        if (current != null) return current;

        return reconstructArcs();
    }

    /**
     * The goal is visible when it is within sensor range,
     * when the angle difference to it is less than 90 degrees,
     * and when moving there is safe (no obstacles in the straight line.
     *
     * @return true if all the conditions above hold
     */
    private boolean isGoalVisible() {
        double distanceToGoal = distance(coords, goal);
        double headingToGoal = calculateHeading(goal);
        double angleDiff = Math.abs(heading - headingToGoal);

        return distanceToGoal <= sensorRange &&
                angleDiff < Math.PI / 2 &&
                isDestinationSafe(goal);
    }

    /**
     * Constructs an arc for winding that is used steer away from obstacles.
     * The best arc is one that reaches the goal with no obstacles on the way.
     */
    private Arc constructWindingArc(Arc a, boolean useDirectionOfA) {
        if (isGoalVisible()) {
            isWinding = false;
            windAmount = 0;
            return reconstructArcs(goal);
        }
        int sign = (useDirectionOfA ? 1 : -1) * a.getSign();
        double step = sign * Math.PI / sensorDensity;
        double currentHeading = normalizeAngle(heading - sign * Math.PI / 2);
        IntPoint sensor;
        Arc winding, good = null, better = null, best = null;
        for (int i = 0; i < sensorDensity; i++) {
            sensor = getPointTowards(currentHeading, sensorRange);
            winding = reconstructArcs(sensor);
            if (!checkCollision(winding)) {
                good = winding;
                if (!checkCollision(second)) {
                    better = winding;
                    if (!checkCollision(third)) {
                        best = winding;
                    }
                }
            }

            currentHeading = normalizeAngle(currentHeading + step);
        }

        if (best != null) return best;
        else if (better != null) return better;
        else return good;
    }

    /**
     * Determines if it is safe to leave the current obstacle.
     */
    private boolean isSafeToLeave(Arc arc) {
        IntPoint current;
        int numberOfSensorsInQuarterCircle = sensorDensity / 2 + 1;
        int orientation = arc.getSign() > 0 ? 1 : -1;
        double angleDiff = orientation * (Math.PI / 2) / numberOfSensorsInQuarterCircle;

        safetyPoints = new LinkedList<>();
        for (int i = 0; i < numberOfSensorsInQuarterCircle; i++) {
            current = getPointTowards(heading + i * angleDiff, sensorRange);
            safetyPoints.addLast(current);
            if (!isDestinationSafe(current)) return false;
        }

        return true;
    }

    /**
     * Moves on the given arc and returns the difference between the previous and the new headings.
     */
    private double moveOnArc(Arc a) {
        double previousHeading = heading;
        heading = a.moveBy(stepSize);
        moveBy(a.getxChange(), a.getyChange());
        return previousHeading - heading;
    }

    /**
     * Sets all the fields for winding and constructs a winding arc.
     */
    private boolean startWinding(Arc a) {
        constructWindingArc(a, true);
        isWinding = true;
        isUnwinding = false;
        unwinding = null;
        return true;
    }

    /**
     * Moves along a given arc or constructs a new one if this is not good enough.
     * Essentially maeks the robot stick to an obstacle like a bug.
     */
    private boolean bugAlong(Arc current) {
        IntPoint frontPoint = getPointTowards(heading, distanceToClosestObstacle());
        boolean forwardsGood = isDestinationSafe(frontPoint);
        // If going forwards is no good, than we need to wind.
        if (!forwardsGood) {
            Arc winding = constructWindingArc(current, false);
            if (winding == null) return false;
            double amount = moveOnArc(winding);
            windAmount += amount;
            return true;
        }

        moveTowards(heading);
        constructWindingArc(current, false);

        if (isSafeToLeave(first)) {
            isWinding = false;
            if (useUnwind) isUnwinding = true;
            else reconstructArcs();
        }

        return true;
    }

    /**
     * Un-sets all the fields for unwinding and reconstructs the arcs.
     */
    private void stopUnwind() {
        isUnwinding = false;
        unwinding = null;
        windAmount = 0;
        reconstructArcs();
    }

    /**
     * Unwinds by rotating and checking if the unwinding arc is about to collide.
     * Allows the robot move around an obstacle.
     */
    private boolean unwind() {
        if (unwinding == null) {
            while (windAmount > 2 * Math.PI) windAmount -= 2 * Math.PI;
            while (windAmount < -2 * Math.PI) windAmount += 2 * Math.PI;
            IntPoint unwindEnd = getPointTowards(heading + windAmount / 2, sensorRange);
            unwinding = new Arc(coords, unwindEnd, heading);
        }

        if (checkCollision(unwinding))
            return startWinding(unwinding);

        double amount = moveOnArc(unwinding);
        if (amount == 0) {
            stopUnwind();
            return true;
        }
        boolean sign = windAmount > 0;
        windAmount -= amount;

        if ((sign && windAmount <= 0) || (!sign && windAmount >= 0))
            stopUnwind();

        return true;
    }

    /**
     * Move on arcs. This method is called by the move method when the robot's
     * arc field is true.
     */
    private boolean arcMove() {
        Arc current = getCurrentArc();
        if (current == null) return false;

        if (isWinding) return bugAlong(current);

        if (isUnwinding) return unwind();

        if (checkCollision(current)) return startWinding(current);

        heading = current.moveBy(stepSize);
        moveBy(current.getxChange(), current.getyChange());

        return true;
    }

    /**
     * Finds a point V as described in the 'arcs and heading construction' document.
     */
    private IntPoint findArcIntersectingV(IntPoint S, double alpha) {
        double alpha2 = Math.abs(alpha / 2);
        double beta2 = Math.abs(SPECIAL * alpha2);
        double SG = distance(S, goal);
        double sinGamma = Math.sin(Math.PI - alpha2 - beta2);

        // SV is the distance between start point S and intersection point V.
        double SV = SG * Math.sin(beta2) / sinGamma;

        double angleGS = angleWithXAxis(S, goal);
        double angleVS = normalizeAngle(angleGS + (alpha < 0 ? -1 : 1) * alpha2);

        int xDiff = (int) (SV * Math.cos(angleVS));
        int yDiff = (int) (SV * Math.sin(angleVS));

        return new IntPoint(S.x + xDiff, S.y + yDiff);
    }

    /**
     * Finds point V such that the
     * {@link PotentialFieldsRobot#second} and {@link PotentialFieldsRobot#third} {@link Arc}s
     * intersect there.
     */
    private IntPoint getArcConnectionPoint(IntPoint arc1End) {
        double headingAtS = first.getHeadingAtEnd();
        double alpha = angleChangeTo(arc1End, goal, headingAtS);
        IntPoint V;
        // If we are already oriented towards the heading, set V to be the mid point.
        if (alpha == 0)
            V = new IntPoint((arc1End.x + goal.x) / 2, (arc1End.y + goal.y) / 2);
        else V = findArcIntersectingV(arc1End, alpha);

        return V;
    }

    /**
     * creates instances for the next two arcs.
     */
    private void planNextTwoArcs(IntPoint arc1End) {
        IntPoint V = getArcConnectionPoint(arc1End);
        second = new Arc(arc1End, V, first.getHeadingAtEnd());
        third = new Arc(V, goal, second.getHeadingAtEnd());
    }

    /**
     * Evaluate all of the robot's potential movement positions & return the best.
     *
     * @return The most valuable point
     */
    private IntPoint evalPoints(Set<IntPoint> points, IntPoint goal) {
        if (points.isEmpty()) return null;

        double minimum = Double.MAX_VALUE;
        IntPoint point = null;
        for (IntPoint p : points) {
            double value = evalMove(p, goal);
            if (value <= minimum) {
                minimum = value;
                point = p;
            }
        }

        return point;
    }

    /**
     * Get the potential field at point p. The lower the value returned, the better the point is as a move.
     *
     * @param p The point to evaluate
     * @return The value of the point
     */
    private double evalMove(IntPoint p, IntPoint goal) {
        final int FACTOR = 10;  //Everything is divided by 10 because otherwise the numbers get too big
        //Get distances to all visible objects
        double[] obstacleDistances = new double[visibleObstacles.size()];
        for (int i = 0; i < visibleObstacles.size(); i++) {
            //Distance is List to 0 if it's closer than the radius to the obstacle
            double dist = distance(p, visibleObstacles.get(i)) - radius;
            obstacleDistances[i] = dist <= 0 ? 0 : dist / FACTOR;
        }
        double goalDist = (distance(p, goal) - radius) / FACTOR;
        //Calculate field power - x^2 so value gets small as distance decreases
        double goalField = Math.pow(goalDist, 2);
        //obs. field power is sum of all obstacles, and gets v. large as distance decreases and vice versa
        double obsField = 0;
        for (int i = 0; i < visibleObstacles.size(); i++) {
            if (obstacleDistances[i] <= 0) {
                obsField = Double.MAX_VALUE;
                break;
            }
            if (obstacleDistances[i] > sensorRange) continue;

            obsField += Math.pow(Math.E, -1 / ((sensorRange) - obstacleDistances[i])) / (obstacleDistances[i]);
        }
        double goalPotential = FACTOR * goalField;
        double obstaclePotential = ((Math.pow(2 * radius, 2) * 4750 * obsField) / (sensorDensity * sensorRange));
        double historyPotential = getPointValue(p);
        return goalPotential + obstaclePotential + historyPotential;
    }

    /**
     * Get all of the points the robot can move to - the robot moves 10 pixels forward,
     * and can turn upto 12 degrees in either direction to simulate continuous movement.
     */
    public Set<IntPoint> getMovablePoints() {
        Set<IntPoint> movablePoints = new HashSet<>();
        final double DEGREES_BETWEEN = 3;
        double angleBetween = Math.toRadians(DEGREES_BETWEEN);
        double currentAngle = mod(heading - Math.toRadians(MAX_TURN), 2 * Math.PI);
        for (int i = 0; i < 2 * MAX_TURN / DEGREES_BETWEEN; i++) {
            //Only make this a 'movable' point if it does not touch an obstacle
            IntPoint possiblePoint = getPointTowards(currentAngle, stepSize);
            Line2D.Double lineToPoint = new Line2D.Double(coords.x, coords.y, possiblePoint.x, possiblePoint.y);

            //Check if this line intersects an obstacle, and if so, don't add it
            boolean wouldCrash = false;
            for (IntPoint obstacle : visibleObstacles)
                if (distance(obstacle, possiblePoint) <= radius) {
                    wouldCrash = true;
                    break;
                }

            if (intersects(lineToPoint) == null && !wouldCrash)
                movablePoints.add(possiblePoint);

            currentAngle = mod(currentAngle + angleBetween, 2 * Math.PI);
        }

        return movablePoints;
    }

    /**
     * Get a list of all the sample points evenly distributed in a 180-degree arc in front of the robot
     **/
    public Set<IntPoint> getSamplePoints() {
        Set<IntPoint> samplePoints = new HashSet<>(sensorDensity);
        double angleBetween = Math.PI / (sensorDensity - 1);
        double currentAngle = mod(heading - Math.PI / 2, 2 * Math.PI);
        sampleSize = distanceToClosestObstacle(); //Sample size changes based on closest obstacle
        for (int i = 0; i < sensorDensity; i++) {
            //Only make this a 'sample' point if it does not touch an obstacle
            IntPoint possiblePoint = getPointTowards(currentAngle, sampleSize);
            Line2D.Double line = new Line2D.Double(coords.x, coords.y, possiblePoint.x, possiblePoint.y);
            if (intersects(line) == null)
                samplePoints.add(possiblePoint);

            currentAngle += angleBetween;
        }
        return samplePoints;
    }

    /**
     * Get all of the points the robot can move to - the number is equal to the robot's sensor density
     * spread equally in a 180 degree arc in front of the robot. Additionally, calculate if a sensor
     * hits an obstacle and make a note of the collision point
     **/
    public Set<IntPoint> getSensorablePoints() {
        Set<IntPoint> sensorablePoints = new HashSet<>(sensorDensity);
        visibleObstacles = new ArrayList<>();
        double angleBetween = Math.PI / (sensorDensity - 1);
        double currentAngle = mod(heading - Math.PI / 2, 2 * Math.PI);
        for (int i = 0; i < sensorDensity; i++) {
            int sensorRange = this.sensorRange;
            //Check for intersecting obstacles
            IntPoint edge = getPointTowards(currentAngle, sensorRange);
            Line2D.Double sensorLine = new Line2D.Double(new Point(coords.x, coords.y), new Point(edge.x, edge.y));
            IntPoint intersection = intersects(sensorLine);
            if (intersection != null) {
                sensorRange = (int) distance(intersection, coords);
                visibleObstacles.add(intersection);
            }
            sensorablePoints.add(getPointTowards(currentAngle, sensorRange));
            currentAngle += angleBetween;
        }
        return sensorablePoints;
    }

    /**
     * The contents of this method were under the 'else' statement of the intersects method below.
     * I have move it here and optimized it a bit, making it more efficient and reading easier.
     */
    private void complexShapeIntersect(ArrayList<IntPoint> intersections, Renderable obstacle, Line2D.Double line) {
        ArrayList<Integer> xs, ys;
        if (obstacle.getClass() == RenderableRectangle.class) {
            RenderableRectangle rect = (RenderableRectangle) obstacle;
                /* Rectangle is treated like a polygon but since because it's a
                 * different class it has to be handled separately - we've got to construct the
				 * polypoints separately (annoyingly)*/
            xs = new ArrayList<>();
            ys = new ArrayList<>();
            xs.add(rect.bottomLeftX);
            xs.add(rect.bottomLeftX);
            xs.add(rect.bottomLeftX + rect.width);
            xs.add(rect.bottomLeftX + rect.width);

            ys.add(rect.bottomLeftY);
            ys.add(rect.bottomLeftY + rect.height);
            ys.add(rect.bottomLeftY + rect.height);
            ys.add(rect.bottomLeftY);
        }
        else if (obstacle.getClass() == RenderablePolygon.class) {
            xs = ((RenderablePolygon) obstacle).xPoints;
            ys = ((RenderablePolygon) obstacle).yPoints;
        }
        else if (obstacle.getClass() == RenderableOval.class) {
            RenderableOval roval = (RenderableOval) obstacle;
            //ovals are treated as their bounding polygons (90-sided) and they have to be circles
            xs = new ArrayList<>();
            ys = new ArrayList<>();

            for (int i = 0; i < 90; i++) {
                int trigPoint = (int) (roval.width / 2 * Math.cos(i * Math.PI / 45));
                xs.add(roval.centreX + trigPoint);
            }

            for (int i = 0; i < 90; i++) {
                int trigPoint = (int) (roval.width / 2 * Math.sin(i * Math.PI / 45));
                ys.add(roval.centreY + trigPoint);
            }

        }
        else return;

        for (int i = 0; i < xs.size(); i++) {
            Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i),
                    xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));
            IntPoint intersect = getIntersectionPoint(line, obsLine);
            if (intersect != null) intersections.add(intersect);
        }
    }

    /**
     * Get the closest point where this line crosses an obstacle - this varies based on the obstacle type
     * In general, this is achieved by turning the obstacle into a series of lines and calling
     * getIntersectionPoint() on the target line and each of the polygon's lines. Once all intersection
     * points are found, the closest to the robot is returned. It is assumed all polygons are convex.
     */
    private IntPoint intersects(Line2D.Double line) {
        ArrayList<IntPoint> intersections = new ArrayList<IntPoint>();
        for (Renderable obstacle : obstacles) {
            if (obstacle.getClass() == RenderablePolyline.class) {
                ArrayList<Integer> xs = ((RenderablePolyline) obstacle).xPoints;
                ArrayList<Integer> ys = ((RenderablePolyline) obstacle).yPoints;
                for (int i = 0; i < xs.size() - 1; i++) {
                    Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i),
                            xs.get(i + 1), ys.get(i + 1));
                    IntPoint intersect = getIntersectionPoint(line, obsLine);
                    if (intersect != null) intersections.add(intersect);
                }
            }
            else complexShapeIntersect(intersections, obstacle, line);
        }
        return intersections.size() == 0 ? null : lowestDist(intersections);
    }

    /**
     * Get the closest point to the robot's coords
     *
     * @param points A list of point
     * @return The point with the smallest distance from the robot
     **/
    private IntPoint lowestDist(ArrayList<IntPoint> points) {
        int lowest = 0;
        for (int i = 0; i < points.size(); i++) {
            if (distance(points.get(i), coords) < distance(points.get(lowest), coords))
                lowest = i;
        }
        return points.get(lowest);
    }

    /**
     * Changes the robot's coordinate and the coordinates of its picture by the specified values.
     * Used in several methods.
     */
    private void moveBy(int length, int height) {
        coords.x += length;
        coords.y += height;
        if (robotPic == null) {
            robotPicAlt.x = coords.x;
            robotPicAlt.y = coords.y;
        }
        else {
            robotPic.x += coords.x;
            robotPic.y += coords.y;
        }

        countPoint(coords);
    }

    /**
     * Have the robot move along a certain heading
     *
     * @param heading The heading to move along.
     **/
    private void moveTowards(double heading) {
        int length = (int) (stepSize * Math.cos(heading));
        int height = (int) (stepSize * Math.sin(heading));
        moveBy(length, height);
        this.heading = heading;
    }

    /**
     * Get the point 'step' pixels along the given heading from the robot's position
     *
     * @param heading The heading to move along
     * @param step    The distance to travel along that heading
     **/
    private IntPoint getPointTowards(double heading, int step) {
        int length = (int) (step * Math.cos(heading));
        int height = (int) (step * Math.sin(heading));
        return new IntPoint(coords.x + length, coords.y + height);
    }

    /**
     * Find the heading that the robot must move to in order to reach a certain point. If
     * the angle is greater than 60 degrees, truncate it to 60 degrees,=.
     *
     * @param end The destination point
     **/
    private double calculateHeading(IntPoint end) {
        double grad = Math.abs(((double) end.y - (double) coords.y)
                / ((double) end.x - (double) coords.x));
        double angle = Math.atan(grad);

        if (end.x - coords.x < 0) {
            if (end.y - coords.y < 0)
                angle = Math.PI + angle;
            else angle = Math.PI - angle;
        }
        else if (end.y - coords.y < 0)
            angle = (Math.PI * 2) - angle;

        return angle;
    }

    /**
     * Get the distance between two points.
     **/
    public static double distance(IntPoint a, IntPoint b) {
        int xdiff = a.x - b.x, ydiff = a.y - b.y;
        return Math.sqrt(xdiff * xdiff + ydiff * ydiff);
    }

    /**
     * Check if the robot falls within the goal radius.
     **/
    public boolean inGoal() {
        return distance(coords, goal) < goalRadius + radius;
    }

    /**
     * Calculate the intersection point of two lines, or return null if there is no
     * intersection.
     *
     * @param line1 The first line
     * @param line2 The second line
     * @return The point of intersection, or null.
     */
    private static IntPoint getIntersectionPoint(Line2D.Double line1, Line2D.Double line2) {
        if (!line1.intersectsLine(line2)) return null;
        double px = line1.getX1(),
                py = line1.getY1(),
                rx = line1.getX2() - px,
                ry = line1.getY2() - py;
        double qx = line2.getX1(),
                qy = line2.getY1(),
                sx = line2.getX2() - qx,
                sy = line2.getY2() - qy;

        double det = sx * ry - sy * rx;
        if (det == 0) {
            return null;
        }
        else {
            double z = (sx * (qy - py) + sy * (px - qx)) / det;
            if (z == 0 || z == 1) return null;  // intersection at end point
            return new IntPoint(
                    (int) (px + z * rx), (int) (py + z * ry));
        }
    }

    /**
     * Calculate a % b, but always result in a positive answer - java's default syntax returns a
     * negative number if the dividend is negative, which is unhelpful when my calculations are
     * performed between 0 and 2PI, rather than -PI and PI.
     **/
    private static double mod(double a, double b) {
        return ((a % b) + b) % b;
    }

    /**
     * Find the distance from the robot to the closest visible obstacle, or some default if
     * none are visible
     **/
    private int distanceToClosestObstacle() {
        if (visibleObstacles.size() == 0) return sampleSizeDefault;
        int closest = 0;
        for (int i = 0; i < visibleObstacles.size(); i++)
            if (distance(coords, visibleObstacles.get(i)) < distance(coords, visibleObstacles.get(closest)))
                closest = i;
        return (int) Math.round(distance(coords, visibleObstacles.get(closest)));
    }

    public IntPoint getPosition() {
        return coords;
    }

    /**
     * Return the image representing the robot. This can be a red blob, or a picture depending on the options
     * enabled.
     **/
    public Renderable getImage() {
        if (robotPic != null) {
            robotPic.x = coords.x - radius;
            robotPic.y = coords.y - radius;
            return robotPic;
        }
        else return robotPicAlt;
    }

    public int getStepSize() {
        return this.stepSize;
    }

}
