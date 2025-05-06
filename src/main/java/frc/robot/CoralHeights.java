package frc.robot;

/**
 * The CoralHeights class represents functionality related to height measurements
 * or elevation control for the robot.
 * 
 * This class handles storing the different height values for the reef.
 */
public class CoralHeights {
    private double[] leftPositions;
    private double[] rightPositions;

    public CoralHeights(double[] leftPositions, double[] rightPositions) {
        this.leftPositions = leftPositions;
        this.rightPositions = rightPositions;
    }

    public double[] getLeftPositions() {
        return leftPositions;
    }

    public double[] getRightPositions() {
        return rightPositions;
    }
}
