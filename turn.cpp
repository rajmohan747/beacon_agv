#include <iostream>
#include <stdio.h>      /* printf */
#include <cmath>
#include <math.h>       /* atan */
#include <unistd.h>

#define PI 3.14159265
using namespace std;


float error=0;
double eq=0;
double xA,xB,yA,yB,target_angle,error_angle;
float navigation_current_heading=0;
double threshold_line = 1;  //  limit within which line condition is considered to be valid
double Threshold = 5;       //  Permissible angular threshold
int target_reached = 0;     //Flag set to check if AGV reached final target
double delta_angle=0;    // AGV steering angle
double Kp = 1;           //Constant parameter for relating error angle with delta angle
double lin_vel=0.5;       //linear velocity of AGV

double steering_current_heading =0;    //Parameters for steering angle
double Threshold_steering=0;

int current_x=0;               // X and Y parameters
int current_y=0;
int next_x=0;
int next_y=0;
int x_limit=10;
int y_limit=10;

double current_position=0;
double position=0;

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


double pointPosition(double x1,double y1,double x2,double y2,double x3,double y3)
{
    double m = (y2-y1)/(x2-x1);
    double c = y1 - (m*x1);
    cout << "y = " << m << "x + " << c <<endl;
    // cout << m << "  " << c << endl;

    double eq= (m*x3) + (c);

    if (y3 == eq)
    {
        cout << "On the line";
        current_position =0;
    }
    else if (y3 > eq)
    {
        cout <<"Above the line";
        current_position =  1;
    }
    else
    {
        cout <<"Below the line";
        current_position =2;
    }

    return current_position;

}

double FindDistanceBtwPoints(double x1,double y1,double x2,double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y2-y1)*(y2-y1));  // Returns Distance Between Two Points (X1,Y1) and (X2,Y2)
}


double AngleBtwLines(double x1, double y1, double x2, double y2, double x3, double y3)
{

    /*double m1=(y2-y1)/(x2-x1);          // Slope of Line AB
    double m2=(y3-y2)/(x3-x2);          // Slope of Line BC
    //cout << "m2 " << m2 << "\n"<<endl;
    if(m1==INFINITY)
    {
        return 90-abs(atan(m2)*(180/PI));

    }
    else if (m2==INFINITY)
    {
        //cout<<90-abs(atan(m1)*(180/PI))<<"\n"<<endl;
        return atan(m1)-90;
    }
    else if (abs(m1*m2)==1)
        return 90;

    double angle=90-atan(abs(m2-m1/(1-(m2*m1))))*180/PI ;
    //cout<<"angle "<< angle <<"\n"<<endl;
    return angle;*/
    double angle1 = atan2(y1-y2, x1-x2);
    double angle2 = atan2(y2-y3, x2-x3);
    double result = (angle2-angle1) * 180 / 3.14;
    if (result<0) {
        result+=360;
    }
    return abs(180-result);
}

double FindTurningDistance(double x1, double y1, double x2, double y2, double x3, double y3,double R)
{
    /* Given Point A(x1,y1), B(x2,y2), and goal C(pointX,pointY) this functions find Intermediate
       point I at distance D from B where the Forklift will start to Turn */
    double angle=AngleBtwLines(x1,y1,x2,y2,x3,y3);
    cout<<"angle  "<<angle<<"\n"<<endl;
    double distance= R/tan(angle/2);
    return distance;
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

void drive(float linear,float angular)  //Will add once switch to ROS
{
//    vel_msg.linear.x=linear;
//    vel_msg.angular.z=angular;
//    vel_pub.publish(vel_msg);

}

bool within_tolerance(double A,double B, double tolerance)
{
    if((A<B+tolerance) && (A>B-tolerance))
    {
        return true;
    }
    else
        return false;
}

bool within_tolerance(double A,double B)
{
    if(A<B)
    {
        return true;
    }
    else
        return false;
}

double imu_delta_map(double del_imu)
{
    double delta=0;
    //cout <<"del_imu  "<< del_imu<<endl;
    if (abs(del_imu) >0)
    {
        del_imu=del_imu;
        delta= 0.77*del_imu;

    }
    //cout<<"Delta" << delta << "\n" <<endl;
    return (delta);
}

int main()
{
    double distance;
    double lineX1, lineY1, lineX2, lineY2,lineX3,lineY3, pointX, pointY;
    lineX1 = 0;            //X1, Y1 is the previous point (A).
    lineY1 = 0;

    lineX2 = 10;            //X2, Y2 is the point we are moving towards (B)
    lineY2 = 0;

    lineX3= 20;
    lineY3= 0;

    pointX = 5;            //PointX , PointY is the point we are to turn Towards
    pointY = -4;

    double radius=10;     // Radius is the set turning radius

    double turning_distance=FindTurningDistance(lineX1,lineY1,lineX2,lineY2,lineX3,lineY3,radius);
    double tolerance= 10; // Set tolerance for reaching goal

    cout<<"Turning Distance"<<turning_distance<<"\n"<<endl;
    cout<<"Delta "<< imu_delta_map(45) << "\n" <<endl;
    cout<<"Distance From Line" << FindDistanceToSegment(lineX1,lineY1,lineX2,lineY2,pointX,pointY)<<"\n"<<endl;
    double intermediate_reached=0; //intermediate between AB
    double intermediate_target_reached=0;


    while(intermediate_target_reached!=1) // While I have not reached target
    {
        // update all variable A,B,C,current position,current heading
        double dist_from_B=FindDistanceBtwPoints(lineX2,lineY2,current_x,current_y);
        double dist_from_AB=FindDistanceToSegment(lineX1,lineY1,lineX2,lineY2,current_x,current_y);
        double dist_from_BC=FindDistanceToSegment(lineX2,lineY2,lineX3,lineY3,current_x,current_y);
        if(!within_tolerance(dist_from_AB,threshold_line)&& intermediate_reached !=1)
        {
            //Correct Course
        }

        // Check When to Start Turning
        if(within_tolerance(dist_from_AB,threshold_line) && !within_tolerance(dist_from_B,turning_distance,Threshold) && intermediate_reached!=1 )
        {
            //Keep moving Forward
            //drive(forward,0);
        }
        else if(within_tolerance(dist_from_AB,threshold_line) && within_tolerance(dist_from_B,turning_distance,Threshold))
        {
                intermediate_reached=1;
            //Start Turning from Intermediate point
            //drive(forward,angle);
        }
        // Check if you have merged with the goal orientation

        /*if((intermediate_reached==1) && within_tolerance(navigation_current_heading,//slope of AC) && within_tolerance(dist_from_B,turning_distance)&&
           within_tolerance(dist_from_BC,threshold_line))
        {
                //target reached
                intermediate_target_reached=1;
                // Start Moving Forward
                //drive(forward,angular speed);
        }
        else if (intermediate_reached)
        {
                //keep turning & check it turning at right delta

        }
        */

    }

    return 0;
}

