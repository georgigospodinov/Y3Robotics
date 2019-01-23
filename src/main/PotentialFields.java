package main;

import dataStructures.RRTree;
import easyGui.EasyGui;
import geometry.IntPoint;
import renderables.*;

import java.awt.*;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

import static main.PotentialFieldsRobot.distance;

public class PotentialFields {

    private static final int FRAME_LENGTH = 1200;
    private static final int FRAME_HEIGHT = 900;
    private static final int GRAPHICS_HEIGHT = 700;
    private static final Random RAND = new Random();

    private final EasyGui gui;

    private final int buttonId;
    private final int circleSId;
    private final int circleLId;
    private final int squareSId;
    private final int squareLId;
    private final int randomLineId;
    private final int clearObsId;
    private final int easyCourseId;
    private final int medCourseId;
    private final int hardCourseId;
    private final int steepestGradientId;
    private final int fractionalProgressId;
    private boolean fractional;
    private final int enMikeId;
    private final int disMikeId;
    private boolean mike;
    private final int enUnwindId;
    private final int disUnwindId;
    private boolean unwind;

    private final int startXId;
    private final int startYId;
    private final int goalXId;
    private final int goalYId;
    private final int goalRadiusId;
    private final int robotRadiusId;
    private final int robotSensorRangeId;
    private final int robotSensorDensityId;
    private final int robotOrientationId;
    private final int robotSpeedId;

    private final int freeSpaceId;
    private final int multiCircleId;
    private final int tunnelsId;
    private final int mazeId;
    private final int shallowCId;
    private final int deepCId;
    private final int severeCId;

    private ArrayList<Renderable> obstacles;

    public PotentialFields() {
        int r;
        //Set up the GUI, labels, buttons
        gui = new EasyGui(FRAME_LENGTH, FRAME_HEIGHT);

        buttonId = gui.addButton(3, 10, "Go Little Robot!", this, "buttonAction");
        gui.addButton(2, 5, "Clear Fields", this, "clearButtonAction");

        gui.addLabel(0, 0, "Starting X:");
        startXId = gui.addTextField(0, 1, null);
        gui.addLabel(1, 0, "Starting Y:");
        startYId = gui.addTextField(1, 1, null);

        gui.addLabel(0, 2, "Goal X:");
        goalXId = gui.addTextField(0, 3, null);
        gui.addLabel(1, 2, "Goal Y:");
        goalYId = gui.addTextField(1, 3, null);

        gui.addLabel(0, 4, "Goal Radius:");
        goalRadiusId = gui.addTextField(0, 5, null);

        gui.addLabel(0, 6, "Robot Radius:");
        robotRadiusId = gui.addTextField(0, 7, null);

        gui.addLabel(1, 6, "Robot Sensor Range:");
        robotSensorRangeId = gui.addTextField(1, 7, null);

        gui.addLabel(0, 8, "Robot Speed (moves/second):");
        robotSpeedId = gui.addTextField(0, 9, null);

        gui.addLabel(1, 8, "Robot Sensor Density:");
        robotSensorDensityId = gui.addTextField(1, 9, null);

        gui.addLabel(2, 8, "Robot Orientation:");
        robotOrientationId = gui.addTextField(2, 9, null);

        gui.addLabel(0, -1, "Leave fields blank for random values!");
        clearObsId = gui.addButton(1, -1, "Clear Obstacles", this, "clearObs");

        r= 3;  // Pre-made courses on row 3
        gui.addLabel(r, 0, "Select a pre-made course: ");
        easyCourseId = gui.addButton(r, 1, "Easy", this, "easyCourse");
        medCourseId = gui.addButton(r, 2, "Medium", this, "medCourse");
        hardCourseId = gui.addButton(r, 3, "Hard", this, "hardCourse");

        r = 4; // Custom courses on row 4
        gui.addLabel(r, 0, "Select a custom course: ");
        freeSpaceId = gui.addButton(r, 1, "Free Space", this, "freeSpace");
        multiCircleId = gui.addButton(r, 2, "Multiple Circles", this, "multiCircle");
        tunnelsId = gui.addButton(r, 3, "Tunnels", this, "tunnels");
        mazeId = gui.addButton(r, 4, "Maze", this, "maze");
        shallowCId = gui.addButton(r, 5, "Shallow C", this, "shallowC");
        deepCId = gui.addButton(r, 6, "Deep C", this, "deepC");
        severeCId = gui.addButton(r, 7, "Severe C", this, "severeC");

        r = 5;  // Custom obstacles on row 5
        gui.addLabel(r, 0, "Or add in your own obstacles: ");
        circleSId = gui.addButton(r, 1, "Circle (S)", this, "genCircleS");
        circleLId = gui.addButton(r, 2, "Circle (L)", this, "genCircleL");
        squareSId = gui.addButton(r, 3, "Square (S)", this, "genSquareS");
        squareLId = gui.addButton(r, 4, "Square (L)", this, "genSquareL");
        randomLineId = gui.addButton(r, 5, "Line", this, "genLine");

        r = 6;  // More options on row 6
        gui.addLabel(r, 0, "Planner: ");
        steepestGradientId = gui.addButton(r, 1, "Steepest Gradient", this, "switchMode");
        fractionalProgressId = gui.addButton(r, 2, "Arcs Based", this, "switchMode");
        gui.setButtonEnabled(steepestGradientId, false);
        fractional = false;

        gui.addLabel(r, 4, "Mike Mode: ");
        enMikeId = gui.addButton(r, 5, "On", this, "switchMike");
        disMikeId = gui.addButton(r, 6, "Off", this, "switchMike");
        gui.setButtonEnabled(disMikeId, false);
        mike = false;

        gui.addLabel(r, 8, "Robot should unwind:");
        enUnwindId = gui.addButton(r, 9, "Yes", this, "switchUnwind");
        disUnwindId = gui.addButton(r, 10, "No", this, "switchUnwind");
        gui.setButtonEnabled(disUnwindId, false);
        unwind = false;

        obstacles = new ArrayList<>();

    }

    public void switchMike() {
        mike = !mike;
        gui.setButtonEnabled(enMikeId, !mike);
        gui.setButtonEnabled(disMikeId, mike);
    }

    public void switchMode() {
        fractional = !fractional;
        gui.setButtonEnabled(fractionalProgressId, !fractional);
        gui.setButtonEnabled(steepestGradientId, fractional);
    }

    public void switchUnwind() {
        unwind = !unwind;
        gui.setButtonEnabled(enUnwindId, !unwind);
        gui.setButtonEnabled(disUnwindId, unwind);
    }

    /**
     * Action when 'clear fields' button is pressed - reset all text fields in the gui.
     **/
    public void clearButtonAction() {
        gui.setTextFieldContent(startXId, "");
        gui.setTextFieldContent(startYId, "");
        gui.setTextFieldContent(goalXId, "");
        gui.setTextFieldContent(goalYId, "");
        gui.setTextFieldContent(goalRadiusId, "");
        gui.setTextFieldContent(robotRadiusId, "");
        gui.setTextFieldContent(robotSensorRangeId, "");
        gui.setTextFieldContent(robotSensorDensityId, "");
        gui.setTextFieldContent(robotSpeedId, "");
        gui.setTextFieldContent(robotOrientationId, "");
    }

    private void setStartPosition(int x, int y, int orientation) {
        setStartPosition(x, y, orientation, 100);
    }

    private void setStartPosition(int x, int y, int orientation, int sensorRange) {
        gui.setTextFieldContent(startXId, String.valueOf(x));
        gui.setTextFieldContent(startYId, String.valueOf(y));
        gui.setTextFieldContent(robotOrientationId, String.valueOf(orientation));
        gui.setTextFieldContent(robotSensorRangeId, String.valueOf(sensorRange));
    }

    private void setGoalPosition(int x, int y) {
        gui.setTextFieldContent(goalXId, String.valueOf(x));
        gui.setTextFieldContent(goalYId, String.valueOf(y));
    }

    /**
     * Set up the 'easy' premade course
     */
    public void easyCourse() {
        setStartPosition(100, 100, 0);
        setGoalPosition(1000, 500);

        RenderableOval r = new RenderableOval(FRAME_LENGTH / 2, GRAPHICS_HEIGHT / 2, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    /**
     * Set up the 'medium' premade course
     */
    public void medCourse() {
        setStartPosition(0, 0, 0);
        setGoalPosition(1400, 700);

        RenderableRectangle r = new RenderableRectangle(150, 150, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
        r2.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r3 = new RenderableRectangle(700, 425, 150, 150);
        r3.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
        r4.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
        r5.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r6 = new RenderableRectangle(1000, 600, 150, 150);
        r6.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r7 = new RenderableRectangle(900, 250, 50, 50);
        r7.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r8 = new RenderableRectangle(1000, 450, 50, 50);
        r8.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r9 = new RenderableRectangle(1150, 350, 50, 50);
        r9.setProperties(Color.DARK_GRAY, 1f, true, false);
        obstacles.add(r);
        obstacles.add(r2);
        obstacles.add(r3);
        obstacles.add(r4);
        obstacles.add(r5);
        obstacles.add(r6);
        obstacles.add(r7);
        obstacles.add(r8);
        obstacles.add(r9);
        gui.draw(obstacles);
        gui.update();
    }

    /**
     * Set up the 'hard' premade course
     */
    public void hardCourse() {
        setStartPosition(0, 0, 0);
        setGoalPosition(1500, 950);
        gui.setTextFieldContent(goalRadiusId, "20");

        RenderableRectangle r = new RenderableRectangle(150, 200, 150, 150);
        r.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
        r2.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
        r4.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
        r5.setProperties(Color.DARK_GRAY, 1f, true, false);
        RenderableRectangle r7 = new RenderableRectangle(900, 350, 50, 50);
        r7.setProperties(Color.DARK_GRAY, 1f, true, false);
        obstacles.add(r);
        obstacles.add(r2);
        obstacles.add(r4);
        obstacles.add(r5);
        obstacles.add(r7);
        gui.draw(obstacles);
        gui.update();

        IntPoint p0 = new IntPoint(800, 700);
        IntPoint p1 = new IntPoint(800, 900);
        IntPoint p2 = new IntPoint(1300, 800);
        IntPoint p3 = new IntPoint(1400, 300);
        IntPoint p4 = new IntPoint(1200, 350);
        RenderablePolyline line = new RenderablePolyline();
        line.addPoint(p0.x, p0.y);
        line.addPoint(p1.x, p1.y);
        line.addPoint(p2.x, p2.y);
        line.addPoint(p3.x, p3.y);
        line.addPoint(p4.x, p4.y);
        line.setProperties(Color.DARK_GRAY, 2f);
        obstacles.add(line);
        gui.draw(line);
        gui.update();
    }

    /* CUSTOM COURSES: */

    /** A Simple example in free space. */
    public void freeSpace() {
        setStartPosition(800, 600, 0, 300);
        setGoalPosition(100, 200);
        clearObs();
    }

    public void multiCircle() {
        setStartPosition(100, 600, 225);
        setGoalPosition(800, 100);

        RenderableOval o = new RenderableOval(200, 200, 300, 300);
        o.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(o);

        o = new RenderableOval(400, 350, 150, 150);
        o.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(o);

        o = new RenderableOval(700, 500, 300, 300);
        o.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(o);

        o = new RenderableOval(600, 200, 100, 100);
        o.setProperties(Color.DARK_GRAY, 1f, true);
        obstacles.add(o);

        gui.draw(obstacles);
        gui.update();
    }

    /**
     * This method creates a grey line with the given coordinates
     * and adds it to the obstacles collection so that it can be displayed by the gui object.
     */
    private void greyLine(int x1, int y1, int x2, int y2) {
        RenderablePolyline line = new RenderablePolyline();
        line.addPoint(x1, y1);
        line.addPoint(x2, y2);
        line.setProperties(Color.DARK_GRAY, 2f);
        obstacles.add(line);
    }

    public void tunnels() {
        setStartPosition(200, 100, 90);
        gui.setTextFieldContent(robotSpeedId, "200");
        gui.setTextFieldContent(robotSensorDensityId, "19");
        setGoalPosition(1200, 600);

        greyLine(100, 100, 100, 800);
        greyLine(300, 0, 300, 600);
        greyLine(500, 100, 500, 800);
        greyLine(700, 0, 700, 600);
        greyLine(900, 100, 900, 800);
        greyLine(1100, 0, 1100, 600);

        gui.draw(obstacles);
        gui.update();
    }

    public void maze() {
        int startY = 800, diff = 100, numberOfRows = 3;
        setStartPosition(50, startY - diff / 2 + 30, 180, 60);
        gui.setTextFieldContent(robotRadiusId, "10");
        gui.setTextFieldContent(robotSpeedId, "200");
        gui.setTextFieldContent(robotSensorDensityId, "181");
        setGoalPosition(1200, startY - diff * (numberOfRows + 1) * 2);

        //horizontal lines
        greyLine(100, startY, 1100, startY);
        greyLine(100, startY - diff, 1000, startY - diff);
        for (int i = 0; i < numberOfRows; i++) {
            greyLine(200, startY - (i + 1) * 2 * diff, 1100, startY - (i + 1) * 2 * diff);
            greyLine(100, startY - (i + 1) * 2 * diff - diff, 1000, startY - (i + 1) * 2 * diff - diff);
        }

        //vertical lines
        greyLine(100, startY - diff, 100, startY - diff * (numberOfRows + 1) * 2 + diff);
        greyLine(1100, startY, 1100, startY - diff * (numberOfRows + 1) * 2 + diff);

        gui.draw(obstacles);
        gui.update();
    }

    public void shallowC() {
        setStartPosition(1000, 100, 90);
        gui.setTextFieldContent(robotSensorDensityId, "70");
        setGoalPosition(100, 600);
        greyLine(400, 50, 500, 500);
        greyLine(500, 500, 900, 600);
        gui.draw(obstacles);
        gui.update();
    }

    public void deepC() {
        setStartPosition(100, 600, 270);
        setGoalPosition(1000, 100);
        greyLine(400, 800, 600, 550);
        greyLine(600, 550, 550, 300);
        greyLine(550, 300, 300, 100);
        greyLine(300, 100, 50, 150);
        gui.draw(obstacles);
        gui.update();
    }

    public void severeC() {
        setStartPosition(100, 100, 180);
        setGoalPosition(1000, 600);

        greyLine(600,400,900,500);
        greyLine(900,500,750,300);
        gui.draw(obstacles);
        gui.update();
    }

	/*Methods to generate random obstacles - circles, squares and lines*/

    public void genCircleS() {
        IntPoint centre = randomPoint(FRAME_LENGTH, FRAME_HEIGHT);
        RenderableOval o = new RenderableOval(centre.x, centre.y, 50, 50);
        o.setProperties(Color.MAGENTA, 1f, true);
        obstacles.add(o);
        gui.draw(o);
        gui.update();
    }

    public void genCircleL() {
        IntPoint centre = randomPoint(FRAME_LENGTH, FRAME_HEIGHT);
        RenderableOval o = new RenderableOval(centre.x, centre.y, 150, 150);
        o.setProperties(Color.MAGENTA, 1f, true);
        obstacles.add(o);
        gui.draw(o);
        gui.update();
    }

    public void genSquareS() {
        IntPoint origin = randomPoint(FRAME_LENGTH - 50, FRAME_HEIGHT - 5);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 50, 50);
        r.setProperties(Color.CYAN, 1f, true, false);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    public void genSquareL() {
        IntPoint origin = randomPoint(FRAME_LENGTH - 150, FRAME_HEIGHT - 150);
        RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 150, 150);
        r.setProperties(Color.CYAN, 1f, true, false);
        obstacles.add(r);
        gui.draw(r);
        gui.update();
    }

    public void genLine() {
        IntPoint p1 = randomPoint(FRAME_LENGTH - 200, FRAME_HEIGHT - 200);
        IntPoint p2 = randomPoint(FRAME_LENGTH - 200, FRAME_HEIGHT - 200);
        RenderablePolyline p = new RenderablePolyline();
        p.addPoint(p1.x, p1.y);
        p.addPoint(p2.x, p2.y);
        p.setProperties(Color.ORANGE, 2f);
        obstacles.add(p);
        gui.draw(p);
        gui.update();

    }

    /**
     * Clear all obstacles from the screen
     **/
    public void clearObs() {
        gui.unDraw(obstacles);
        obstacles = new ArrayList<Renderable>();
        gui.update();
    }

    /**
     * Start the GUI
     **/
    public void runRobot() {
        gui.show();
    }

    /**
     * This method checks if a given field contains valid input
     * if true it returns that input
     * otherwise it returns a random value within [min, min+range].
     */
    private int parseInput(int fieldID, int range, int min) {
        String input = gui.getTextFieldContent(fieldID);
        int value;
        try {
            value = Integer.parseInt(input);
            if (value < 0)
                throw new NumberFormatException("Negative values are not accepted");
        }
        catch (NumberFormatException e) {
            value = RAND.nextInt(range) + min;
        }
        gui.setTextFieldContent(fieldID, String.valueOf(value));

        return value;
    }

    /**
     * Get the parameters from the text fields and use these to set the robot moving
     **/
    public void buttonAction() throws InterruptedException {
        int startX = parseInput(startXId, FRAME_LENGTH, 0);
//        int startX = 100;
        int startY = parseInput(startYId, FRAME_HEIGHT, 0);
//        int startY = 100;
        int goalX = parseInput(goalXId, FRAME_LENGTH, 0);
//        int goalX = 500;
        int goalY = parseInput(goalYId, FRAME_HEIGHT, 0);
//        int goalY = 100;
        int goalRadius = parseInput(goalRadiusId, 70, 30);
        int robotRadius = parseInput(robotRadiusId, 15, 20);
        int robotSensorRange = parseInput(robotSensorRangeId, 300, 100);
        int robotSensorDensity = parseInput(robotSensorDensityId, 300, 100);
//        int robotSensorDensity = 181;
//        int robotSpeed = parseInput(robotSpeedId, 1, 40);
        int robotSpeed = 200;
//        int robotOrientation = parseInput(robotOrientationId, 360, 0);
        int robotOrientation = 30;

        String image = mike ? "mike.png" : null;

        IntPoint start = new IntPoint(startX, startY);
        IntPoint goal = new IntPoint(goalX, goalY);
        goLittleRobot(start, goal, goalRadius, robotRadius, robotSensorRange, robotSensorDensity, robotSpeed, image, robotOrientation);
    }

    private void setButtonsEnabled(boolean isEnabled) {
        gui.setButtonEnabled(buttonId, isEnabled);
        gui.setButtonEnabled(circleSId, isEnabled);
        gui.setButtonEnabled(circleLId, isEnabled);
        gui.setButtonEnabled(squareSId, isEnabled);
        gui.setButtonEnabled(squareLId, isEnabled);
        gui.setButtonEnabled(randomLineId, isEnabled);
        gui.setButtonEnabled(clearObsId, isEnabled);
        gui.setButtonEnabled(easyCourseId, isEnabled);
        gui.setButtonEnabled(medCourseId, isEnabled);
        gui.setButtonEnabled(hardCourseId, isEnabled);
        gui.setButtonEnabled(freeSpaceId, isEnabled);
        gui.setButtonEnabled(multiCircleId, isEnabled);
        gui.setButtonEnabled(tunnelsId, isEnabled);
        gui.setButtonEnabled(mazeId, isEnabled);
        gui.setButtonEnabled(shallowCId, isEnabled);
        gui.setButtonEnabled(deepCId, isEnabled);
        gui.setButtonEnabled(severeCId, isEnabled);
    }

    private void drawPathFromStart(RenderablePolyline path, PotentialFieldsRobot rob, RRTree startAndGoal) {
        path.addPoint(rob.getPosition().x, rob.getPosition().y);
        gui.clearGraphicsPanel();
        gui.draw(startAndGoal);
        gui.draw(path);
        gui.draw(obstacles);
        drawRobot(rob);
    }

    private RenderableString showCurrentPath(RenderableString rs, int l) {
        gui.unDraw(rs);
        rs = new RenderableString(820, 20, "Distance Travelled (pixels): " + l);
        rs.setLayer(456);
        rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
        gui.draw(rs);

        return rs;
    }

    private RenderableString showCurrentSmoothness(RenderableString rs2, RenderablePolyline path) {
        gui.unDraw(rs2);
        rs2 = new RenderableString(820, 0, "Path Smoothness Rating: " + (calculateSmoothness(path)));
        rs2.setLayer(456);
        rs2.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
        gui.draw(rs2);

        return rs2;
    }

    private void showStuckMessage() {
        RenderableString stuck = new RenderableString(500, 500, "I'm Stuck :(");
        stuck.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 32));
        stuck.setLayer(456);
        gui.draw(stuck);
        gui.update();
    }

    private void initialSetup(RRTree startAndGoal, RenderablePolyline path, PotentialFieldsRobot rob) {
        gui.clearGraphicsPanel();
        gui.draw(startAndGoal);
        gui.draw(path);
        gui.draw(obstacles);
        drawRobot(rob);
        gui.update();
    }

    /**
     * Set the robot moving towards a goal on the screen with set radius, step size, etc.
     *
     * @param start              The coordinates of the starting point
     * @param goal               The coordinates of the goal
     * @param goalRad            The radius of the goal - if the robot falls within this, it wins
     * @param robotRadius        The width of the robot
     * @param robotSensorRange   How far the robot can 'see'
     * @param robotSensorDensity The number of sensor lines the robot can use
     * @param robotSpeed         The number of moves per second
     */
    public void goLittleRobot(IntPoint start, IntPoint goal, int goalRad,
                              int robotRadius, int robotSensorRange, int robotSensorDensity,
                              int robotSpeed, String image, int robotOrientation) throws InterruptedException {
        setButtonsEnabled(false);

        //Create the robot, start & end points, renderables
        PotentialFieldsRobot rob = new PotentialFieldsRobot(image, start, goal, robotRadius,
                robotSensorRange, robotSensorDensity, goalRad, obstacles, robotOrientation, fractional, unwind);
        RRTree startAndGoal = new RRTree(Color.black);
        startAndGoal.setStartAndGoal(start, goal, goalRad);
        RenderableString distTraveled = null;
        RenderableString smoothness = null;
        RenderablePolyline path = new RenderablePolyline();
        path.setProperties(Color.BLACK, 1f);
        path.addPoint(start.x, start.y);

        initialSetup(startAndGoal, path, rob);

        int l = 0;
        //Loop until the robot reaches the goal or gets stuck
        while (!rob.inGoal()) {
            Thread.sleep(1000 / robotSpeed);
            boolean moved = rob.move(); //Move 1 step

            //If not moved rob is stuck.
            if (!moved) {
                showStuckMessage();
                break;
            }

            drawPathFromStart(path, rob, startAndGoal);

            l += rob.getStepSize();
            distTraveled = showCurrentPath(distTraveled, l);
            smoothness = showCurrentSmoothness(smoothness, path);
            gui.update();
        }

        //Print metrics to console for evaluation purposes.
        System.out.println("Start Point: " + start + "\tEnd Point: " + goal);
        System.out.println(String.format("Crow's flight:%.2f", distance(start, goal)));
        System.out.println("With: " + (fractional ? "Arcs" : "Forwards"));
        System.out.println("Robot got to: " + rob.getPosition());
        System.out.println("Distance Travelled (pixels): " + l);
        System.out.println("Path Smoothness: " + calculateSmoothness(path) + "\n");

        setButtonsEnabled(true);
    }

    /**
     * Draw the robot, it's sensors (in green), and all of the points it can move to (in blue)
     */
    private void drawRobot(PotentialFieldsRobot rob) {
        gui.draw(rob.getImage());
        try {
            Thread.sleep(1); //necessary when rendering images to give them time to load
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        for (IntPoint p : rob.getSensorablePoints()) {
            RenderablePolyline r = new RenderablePolyline();
            r.addPoint(rob.getPosition().x, rob.getPosition().y);
            r.addPoint(p.x, p.y);
            r.setProperties(Color.GREEN, 1f);
            RenderablePoint pp = new RenderablePoint(p.x, p.y);
            pp.setProperties(Color.GREEN, 5f);
            gui.draw(r);
            gui.draw(pp);
        }
        for (IntPoint p : rob.getSamplePoints()) {
            RenderablePolyline r = new RenderablePolyline();
            r.addPoint(rob.getPosition().x, rob.getPosition().y);
            r.addPoint(p.x, p.y);
            r.setProperties(Color.BLUE, 3f);
            RenderablePoint pp = new RenderablePoint(p.x, p.y);
            pp.setProperties(Color.BLUE, 6f);
            gui.draw(r);
            gui.draw(pp);
        }

        // Show the predicted arc.
        RenderablePolyline arc = new RenderablePolyline();
        LinkedList<IntPoint> arcPoints = rob.getArcPoints();
        for (IntPoint p : arcPoints) arc.addPoint(p.x, p.y);
        arc.setProperties(Color.MAGENTA, 3f);
        gui.draw(arc);

        for (IntPoint p : rob.getSafetyPoints()) {
            RenderablePoint rp = new RenderablePoint(p.x, p.y);
            rp.setProperties(Color.ORANGE, 4f);
            gui.draw(rp);
        }

        for (IntPoint p : rob.getPointsIveBeen()) {
            RenderablePoint rp = new RenderablePoint(p.x, p.y);
//            System.out.println("rp = " + rp);
            rp.setProperties(Color.RED, 10f);
            gui.draw(rp);
        }

    }

    /**
     * Generate a random point in 2D space in the range ([0-maxX,], [0-maxY]) for
     * obstacle creation
     */
    private static IntPoint randomPoint(int maxX, int maxY) {
        Random rand = new Random();
        IntPoint point = new IntPoint();
        point.x = rand.nextInt(maxX + 1);
        point.y = rand.nextInt(maxY + 1);
        return point;
    }

    /**
     * Smoothness metric. 0 = completely smooth, high values = not very smooth
     **/
    private double calculateSmoothness(RenderablePolyline line) {
        if (line.xPoints.size() < 20) return 0;
        double totalDiff = 0;
        for (int i = 0; i < line.xPoints.size() - 20; i += 10) {
            IntPoint p1 = new IntPoint(line.xPoints.get(i), line.yPoints.get(i));
            IntPoint pmid = new IntPoint(line.xPoints.get(i + 10), line.yPoints.get(i + 10));
            IntPoint p2 = new IntPoint(line.xPoints.get(i + 20), line.yPoints.get(i + 20));
            Line2D pline = new Line2D.Double();
            pline.setLine(p1.x, p1.y, p2.x, p2.y);

            double distance = Math.abs((p2.y - p1.y) * pmid.x - (p2.x - p1.x) * pmid.y + p2.x * p1.y - p2.y * p1.x)
                    / Math.sqrt(Math.pow(p2.y - p1.y, 2) + Math.pow(p2.x - p1.x, 2));
            totalDiff += distance;
        }
        return totalDiff / line.xPoints.size();
    }

    public static void main(String[] args) {
        PotentialFields fields = new PotentialFields();
        fields.runRobot();
    }
}
