#include <iostream>
#include <stdio.h>      /* printf */
#include <cmath>
#include <math.h>       /* atan */
#define PI 3.14159265
using namespace std;


float error=0;
double eq=0;
double xA,xB,yA,yB,target_angle,error_angle;
float navigation_current_heading=0;

double angle (double xA,double xB,double yA,double yB)  //angle between two points
{

    double X,Y,param,result;
    X=xB-xA;
    Y=yB-yA;

    param = Y/X;
    result = atan (param) * 180 / PI;

    if(xB>xA)
    {
        if(yB < yA)
        {
            result= 360+result;
        }
        else if (yB > yA)
        {
            result=result;
        }
        else
        {
            result=0;
        }
    }
    else if(xB < xA)
    {
        if(yB>yA)
        {
            result=180+result;
        }
        else if (yB < yA)
        {
            result=180+result;
        }
        else
        {
            result = 180;
        }

    }
    else
    {
        if(yB < yA)
        {
            result=270;
        }
        else
        {
            result=90;
        }
    }
    //cout<<result<<endl;
    return result;
}


float Error(float target_angle)   // Calculating error angle at every instant
{
    if (abs(target_angle - navigation_current_heading) < 180)
    {
        if (navigation_current_heading > target_angle)
        {
            error = navigation_current_heading - target_angle;
        }
        else
        {
            error = target_angle - navigation_current_heading;
        }
    }
    else
    {
        if (navigation_current_heading > target_angle)
        {
            error = 360 - navigation_current_heading + target_angle;
        }
        else
        {
            error = 360 + navigation_current_heading - target_angle;
        }
    }
    return error;
}


void pointPosition(double x1,double y1,double x2,double y2,double x3,double y3)
{
    double m = (y2-y1)/(x2-x1);
    double c = y1 - (m*x1);
    cout << "y = " << m << "x + " << c <<endl;
   // cout << m << "  " << c << endl;

    double eq= (m*x3) + (c);

    if (y3 == eq)
    {
        cout << "On the line";
    }
    else if (y3 > eq)
    {
        cout <<"Above the line";

    }
    else
    {
        cout <<"Below the line";
    }
}




double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}

    int main()
    {
        double distance;
        double lineX1, lineY1, lineX2, lineY2, pointX, pointY;
        lineX1 = 1;            //X1, Y1 are the first point of that line segment.
        lineY1 = 1;

        lineX2 = 6;            //X2, Y2 are the end point of that line segment
        lineY2 = 6;

        pointX = 2;            //pointX, pointY are the point of the reference point.
        pointY = 0;

        pointPosition(lineX1, lineY1, lineX2, lineY2, pointX, pointY);
        cout << ""<<endl;
        distance = FindDistanceToSegment(lineX1, lineY1, lineX2, lineY2, pointX, pointY);       //calling function to find the shortest distance
        cout<<"Distance = "<<distance <<endl;

        target_angle=angle(lineX1,lineX2,lineY1,lineY2);
        error_angle = Error(target_angle);
        cout << "Target Angle "<<target_angle <<endl;
        cout << "Error Angle "<<error_angle <<endl;
        
        
        
        
        
        return 0;
    }

