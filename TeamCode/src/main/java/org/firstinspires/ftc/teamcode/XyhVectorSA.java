package org.firstinspires.ftc.teamcode;

public class XyhVectorSA {
    private double xPosition;
    private double yPosition;
    private double theta;
    private String type;

    public XyhVectorSA(double xPosition, double yPosition, double theta, String type) {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.theta = theta;
        this.type = type;
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

    public String getType() {
        return type;
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
