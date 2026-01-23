package org.firstinspires.ftc.teamcode;

public class XyhVector {

    public double x;   // X position (cm)
    public double y;   // Y position (cm)
    public double h;   // Heading (radians)

    // Constructor
    public XyhVector(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    // Copy constructor
    public XyhVector(XyhVector other) {
        this.x = other.x;
        this.y = other.y;
        this.h = other.h;
    }
}
