package frc.robot.util;

import java.util.ArrayList;

import java.util.Arrays;

import java.util.HashMap;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;



public class LookUpTableShooterAngles {

     List<Integer> distances = Arrays.asList(36,42,48,54,60,66,72,78,84,90,96,102,105,108,113,114,120,126,132,138,144,150,156,162,178,185,400); 

     List<Double> angles = Arrays.asList(-31.0,-23.0,-20.0,-17.0,-14.0,-13.0,-10.0,-8.5,-6.5,-6.0,-3.5,-3.0,-2.5,-1.5,0.7,1.0,1.5,2.0,3.0,3.25,4.25,4.75,5.25,6.0,7.0,7.4,14.0);

        double angle = 0;

       

        int index1 = 0;

        int index2 = 0;

        double percentage =0;


    public LookUpTableShooterAngles(){
    }


    // This uses a regression of the original LUT, that I made with Desmos
    // For more information, look at https://www.desmos.com/calculator/egj3vqmti1
    // public double getAngle(double distance) {
    //     double
    //         a = -1.69243,
    //         b = -0.0160853,
    //         c = 3.59561,
    //         d = 0.138895,
    //         f = 0.998668,
    //         g = -35.9521,
    //         h = 0.0140762,
    //         i = 7.86851;
        
    //     return a*Math.exp(b*distance + c) + d*Math.log(f*distance + g) + h*distance + i;
    // }

    public double getAngle(double distance){

        //Variable Initilization

 double error = Math.abs(distances.get(0)-distance);
       


        if(distance<distances.get(0) || distance>distances.get(distances.size()-1)) {
            
        	return -35.9;

        }

        //Searches for point closest to our distance

        for(int i =0; i<distances.size(); i++){

            if(error>Math.abs(distances.get(i)-distance)){

                error = Math.abs(distances.get(i)-distance);

                index1 = i;

            }

        }

        //Searches for point on the other end of the range our distance falls into

        //Always makes index1 smaller and index2 larger

        if(distance>distances.get(index1)){

            if(index1<distances.size())

            index2 = index1+1;

            else

            return (double) angles.get(index1);

        }

        else if(distance<distances.get(index1)){

            if(index1>0){

            index2 = index1;

            index1=index2-1;

            }

            else

            return (double) angles.get(0);

        }

        //calculates what percent of the way it is between the two distance bounds 

        //and then gets that percent of the way through the corresponding angle bounds
        
            percentage = Math.abs( (distance-distances.get(index1))/(distances.get(index2)-distances.get(index1)));
        
            angle = (double)angles.get(index1)+(((double)angles.get(index2)-(double)angles.get(index1))*percentage);

            return angle;
        


    }

   

}