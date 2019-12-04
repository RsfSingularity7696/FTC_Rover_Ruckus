package org.firstinspires.ftc.teamcode;

public class XYZ {
    private float x = -17;
    private float y = 5;
    private float z = 15;
    private float tolerance = 1;

    public void setX(float x){
        this.x = x;

    }
    public void setY(float y){
        this.y = y;
    }
    public void setZ(float z){
        this.z = z;
    }
    public void setTolerance(float tolerance){
        this.tolerance = tolerance;
    }


    public float getX(){
        return x;
    }
    public float getY(){
        return y;
    }
    public float getZ(){
        return z;
    }
    public float getTolerance(){
        return tolerance;
    }
}
