package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class coordinateObject
{
    //variables
    ArrayList<coordinateObject> history = new ArrayList<>();
    private double x , y;

    double mapX = 57.5, mapY = 86.5;

    public coordinateObject(double startX, double startY)
    {
        x = startX;
        y = startY;
    }

    coordinateObject(double location)
    {
        x = 57.5;
    }

    coordinateObject(String pathName)
    {
        if(pathName.equals("Blue Crater"))
        {
            x = 57.5;
            y = 57.5;
        }
        else if ( pathName.equals( "Blue Depot"))
        {

            x = 57.5;
            y = 86.5;

        }
        else if ( pathName.equals("Red Crater"))
        {
            x = 86.5;
            y = 86.5;
        }
        else if ( pathName.equals( "Red Depot"))
        {
            x = 86.5;
            y = 57.5;

        }
        else
        {
            x = 739172;
            y = 31659;
        }
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public void setX(double x, double y)
    {
        this.x = x;
        this.y = y;
        history.add(coordinateObject);
    }

    public void setY(double y)
    {
        this.y = y;
        history.add(y);
    }

    public String getHistory()
    {
        return history.toString();
    }

    public String toString()
    {
        return ("x,y" + x + y);
    }
}
