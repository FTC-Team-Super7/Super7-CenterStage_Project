package org.firstinspires.ftc.teamcode;

public class Point implements Comparable<Point> {
    public double xP, yP, ang;
    public boolean invertSpline, spline;

    public Point(double xP, double yP, double ang) {
        this.xP = xP;
        this.yP = yP;
        this.ang = ang;
        invertSpline = false;
        spline = false;
    }

    public Point(double xP, double yP) {
        this.xP = xP;
        this.yP = yP;
        invertSpline = false;
        spline = false;
    }

    public Point(double xP, double yP, boolean spline, boolean invertSpline){
        this.xP = xP;
        this.yP = yP;
        this.spline = spline;
        this.invertSpline = invertSpline;
    }

    public void setX(double xP) {
        this.xP = xP;
    }

    public void setY(double yP) {
        this.yP = yP;
    }



    // Returns the distance from this point to any other point specified
    public double getDistance(Point p2) {
        return Math.sqrt((p2.yP - this.yP) * (p2.yP - this.yP) + (p2.xP - this.xP) * (p2.xP - this.xP));
    }

    @Override
    public int compareTo(Point o) {
        if (this.xP < o.xP) return -1;
        if (this.xP == o.xP) return 0;
        return 1;
    }

    @Override
    public String toString() {
        return "{" + "xP=" + xP + "\n, yP=" + yP + "\n, ang=" + ang + '}';
    }
}
