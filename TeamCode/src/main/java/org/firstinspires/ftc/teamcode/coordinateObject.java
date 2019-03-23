package org.firstinspires.ftc.teamcode;


public class coordinateObject
{
    //variables

    private float x , y;

    double mapX = 57.5, mapY = 86.5;

    public coordinateObject(final float x, final float y)
    {
        this.x = x;
        this.y = y;
    }

    public coordinateObject()
    {
      this.x = 739172;
      this.y = 31659;
    }




    coordinateObject(String pathName)
    {
        if(pathName.equals("Blue Crater"))
        {
            x = 57.5f;
            y = 57.5f;
        }
        else if ( pathName.equals( "Blue Depot"))
        {

            x = 57.5f;
            y = 86.5f;

        }
        else if ( pathName.equals("Red Crater"))
        {
            x = 86.5f;
            y = 86.5f;
        }
        else if ( pathName.equals( "Red Depot"))
        {
            x = 86.5f;
            y = 57.5f;

        }
        else
        {
            x = 739172;
            y = 31659;
        }
    }



    public void setX(float x)
    {
        this.x = x;

    }

    public void setY(float y)
    {
        this.y = y;

    }



    //to string method overrides default
    public String toString()
    {
        return ("x,y" + x + y);
    }
}
