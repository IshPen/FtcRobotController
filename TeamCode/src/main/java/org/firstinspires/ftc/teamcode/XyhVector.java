package org.firstinspires.ftc.teamcode;

public class XyhVector {
    private double xPosition;
    private double yPosition;
    private double theta;

    public XyhVector(double xPosition, double yPosition, double theta) {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.theta = theta;
    }

    // Getter methods
    public double getX() {
        return xPosition;
    }

    public double getY() {
        return yPosition;
    }

    public double getTheta() {
        return theta;
    }

    // Setter methods
    public void setX(double xPosition) {
        this.xPosition = xPosition;
    }

    public void setY(double yPosition) {
        this.yPosition = yPosition;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    // Override toString method to provide a string representation of the vector
    @Override
    public String toString() {
        return "XyhVector [xPosition=" + xPosition + ", yPosition=" + yPosition + ", theta=" + theta + "]";
    }
}
