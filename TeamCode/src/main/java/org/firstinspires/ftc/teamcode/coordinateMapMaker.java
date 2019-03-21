package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/*
 *
 */
public class coordinateMapMaker
{
    ArrayList<coordinateObject> history = new ArrayList<>();

    //adds new coordinateObject objects to the arrayList
    public void newCoord (double x, double y)
    {
        coordinateObject newPoint = new coordinateObject(x,y);
        history.add(newPoint);
    }

    public String getHistory()
    {
        return history.toString();
    }

}
