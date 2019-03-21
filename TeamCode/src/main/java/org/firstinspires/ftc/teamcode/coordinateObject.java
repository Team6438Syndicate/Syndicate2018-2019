package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class coordinateObject
{
    //variables

    private double x , y;

    double mapX = 57.5, mapY = 86.5;

    public coordinateObject(final double x, final double y)
    {
        this.x = x;
        this.y = y;
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

    public void setX(double x)
    {
        this.x = x;

    }

    public void setY(double y)
    {
        this.y = y;

    }




    public String toString()
    {
        return ("x,y" + x + y);
    }
}
