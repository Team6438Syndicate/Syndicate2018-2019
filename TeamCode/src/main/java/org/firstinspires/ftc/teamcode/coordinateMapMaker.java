package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/*
 * test for the map of the field
 *
 * Created 3/21/19
 * Author: Matthew Kaboolian
 *
 */
public class coordinateMapMaker extends LinearOpMode
{
   ArrayList<coordinateObject> history = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime etime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive()) {
            telemetry.addData("time right now = ", etime.now(TimeUnit.MILLISECONDS));
            telemetry.update();
            if(etime.now(TimeUnit.MILLISECONDS) % 10 == 0)
            {

            }
        }
    }



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
