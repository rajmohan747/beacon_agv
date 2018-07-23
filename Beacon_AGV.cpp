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
double navigation_current_heading=5;
double threshold_line = 1;  //  limit within which line condition is considered to be valid
double Threshold = 1;       //  Permissible angular threshold
int target_reached = 0;     //Flag set to check if AGV reached final target
double Kp = 1;           //Constant parameter for relating error angle with delta angle
double lin_vel=0.5;       //linear velocity of AGV

//double steering_angle =0;    //Parameters for steering angle

double steering =0;
double Threshold_steering=1;
double delta_angle=0;    // AGV steering angle

int current_x=0;               // X and Y parameters
int current_y=0;
int next_x=0;
int next_y=0;
int x_limit=1;
int y_limit=1;

double current_position=0;
double position=0;
bool node_not_reached = true;
bool temp = false;

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
        current_position =-1;
    }
    return current_position;

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


double delta_angle_calculator(double error_angle,double Kp)
{
    double result;

    if(abs(result) > Threshold_steering)
    {
        result = error_angle * Kp;
    }
    else
    {
        result = 0;
    }
    return result;
}



int main()
{
    double distance;
    double lineX1, lineY1, lineX2, lineY2, pointX, pointY;
    lineX1 = 1;            //X1, Y1 are the first point of that line segment.
    lineY1 = 1;

    lineX2 = 6;            //X2, Y2 are the end point of that line segment
    lineY2 = 1;

    pointX = 2;            //pointX, pointY are the point of the reference point.
    pointY = 0;




    while(1)
    {
        if(target_reached != 1)   //Not used as of now
        {
            //while((abs(next_x-current_x) > x_limit)  ||  (abs(next_y-current_y) > y_limit))    //Condition to check if the robot reached within the boundary of target

            node_not_reached = ((abs(lineX2-pointX) > x_limit)  ||  (abs(lineY2-pointY) > y_limit));
            target_angle=angle(lineX1,lineX2,lineY1,lineY2);
            error_angle = Error(target_angle);

            Kp=pointPosition(lineX1, lineY1, lineX2, lineY2, pointX, pointY);
            delta_angle = delta_angle_calculator(error_angle,Kp) ;      // Experimenting by relating error angle from IMU to delta angle for AGV control


            cout << "Target Angle "<<target_angle <<endl;
            cout << "Error Angle "<<error_angle <<endl;
            cout << (node_not_reached) << endl;

          //  cout<< "Check point3"<<endl;

            //This is a one time process,should not be repeated again unless or until the particular motion execution is not completed
            while (abs(steering - delta_angle) > Threshold_steering)    //Initial steering motion
            {
                drive(0,delta_angle);
           // cout<< "Check point2"<<endl;
            cout<<abs(steering - delta_angle)<<endl;

                //CALL SUBSCRIBER FOR UPDATING steering_current_heading
            }
            //cout<< "Check point"<<endl;
            while (((navigation_current_heading - target_angle) > Threshold) && (!node_not_reached))  // First Curve
            {
             //CHANGE TO node_not_reached later
                drive(lin_vel,0);
                cout << "Dummy"<<endl;
                //CALL SUBSCRIBER FOR UPDATING navigation_current_heading
            }
            drive(0,0);                                                 //Stop
            usleep(2000000);

            delta_angle = delta_angle* -1 ;   //Experimenting with delta angle of same magnitude and opp direction for aligning trajectory with line
            while (abs(steering- delta_angle) > Threshold_steering)    //Steering angle change for countering first curve
            {

                drive(0,delta_angle);
                //CALL SUBSCRIBER FOR UPDATING steering_current_heading
            }

            while ((distance < threshold_line) && ((navigation_current_heading - target_angle) > Threshold))  //Condition to check if the robot trajectory met the straight line
            {
                distance = FindDistanceToSegment(lineX1, lineY1, lineX2, lineY2, pointX, pointY);
                //calling function to find the shortest distance
                //Update pointX and pointY with beacon's output X and Y
                //Update lineX1,lineY1 ,lineX2,lineY2 with the intermediate node points
                drive(lin_vel,0);
                //CALL SUBSCRIBER FOR UPDATING navigation_current_heading
            }

            while (abs(steering - 0) > Threshold_steering)    //Steering angle change for Original position
            {

                drive(0,0);
                //CALL SUBSCRIBER FOR UPDATING steering_current_heading
            }

            while ((distance < threshold_line) && ((navigation_current_heading - target_angle) > Threshold))
                //Even while moving straight make sure you are not deviating too much out of the Trajectory
            {
                distance = FindDistanceToSegment(lineX1, lineY1, lineX2, lineY2, pointX, pointY);
                drive(lin_vel,0);
                //CALL SUBSCRIBER FOR UPDATING navigation_current_heading
            }
        }


    }
    return 0;
}

