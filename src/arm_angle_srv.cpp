#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>
#include "pluto/arm_angles_msg.h"

#include "ikfast61_plutoRightArm.cpp"

#define IKREAL_TYPE IkReal 
const double pi=3.14159;

using namespace std;

ros::ServiceClient client;

pluto::arm_angles_msg srv;

bool findSolution(IKREAL_TYPE eerot[9],IKREAL_TYPE eetrans[3], IKREAL_TYPE current_pose[6], IKREAL_TYPE target_pose[6])
{ 
     struct path_cost
     {
     	int index;
     	double cost;
     };
          
     unsigned int num_of_joints = GetNumJoints();
     unsigned int num_free_parameters = GetNumFreeParameters();
     IkSolutionList<IKREAL_TYPE> solutions;
     std::vector<IKREAL_TYPE> vfree(num_free_parameters);
     std::vector<int> good_solutions;
     
     bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
     if (!bSuccess)
     {
     	std::cout << "No Solutions Found" << std::endl;
     	return bSuccess;
     }
     
     else
     {
     	unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
     	std::cout << num_of_solutions << " Solutions Found" << std::endl;
     	std::vector<IKREAL_TYPE> solvalues(num_of_joints);
     	int this_sol_free_params = 0; 
     	for(std::size_t i = 0; i < num_of_solutions; ++i)  // Get the solutions we like based on the constraints
     	{
     	  const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
          this_sol_free_params = (int)sol.GetFree().size(); 
          std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
          
          sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
          
          if (solvalues[0] <= pi/2 && solvalues[0] >= -pi/2 && solvalues[1] <= 0 && solvalues[2] <= 0 && solvalues[4] <= 0)
	  {
	    good_solutions.push_back(i);
	  }
         /*
          for( std::size_t j = 0; j < solvalues.size(); ++j)
                    printf("%.15f, ", solvalues[j]);
          std::cout << std::endl;
          */
     	}
     	
     	if (good_solutions.size() <= 1) // There is only one good solution so that is the one we choose
     	{
	     	  int good_sol_index = good_solutions[0];
	     	  std::cout << "Good Solution Index " << good_sol_index << std::endl;
	     	  const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(good_sol_index);
	     	  this_sol_free_params = (int)sol.GetFree().size(); 
		  std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
		  
	     	  sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);

	     	  for( std::size_t j = 0; j < solvalues.size(); ++j)
	     	  {
	     	     target_pose[j] = solvalues[j];
		     printf(" target Pos %.15f, ", target_pose[j]);
		  }
		  
		  std::cout << std::endl;
     	}
     	
     	else // There are more than one good solutions. So we need to choose the right one
     	{
     		path_cost costs[good_solutions.size()];
     			
     	   	for (int i=0; i < good_solutions.size(); i++)
	     	{
	     	  path_cost costValue;
	     	  int good_sol_index = good_solutions[i];
	     	  //std::cout << "Good Solution Index " << good_sol_index << std::endl;
	     	  const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(good_sol_index);
	     	  this_sol_free_params = (int)sol.GetFree().size(); 
		  std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
		  
	     	  sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
	     	 
		  double cost = 0.0;
		  int scale = 1;
		  //std::cout << "1 " << solvalues.size() << std::endl;
		  int numjoints = solvalues.size();
	     	  
	     	  for(int j = solvalues.size() - 2; j >= 0; j--)
	     	  {
	     	    
	     	    //std::cout << j << " " << current_pose[j] << " " << solvalues[j] << std::endl;
	     	    cost = cost + scale * abs(current_pose[j] - solvalues[j]);
	     	    scale++;
		    //printf("%.15f, ", solvalues[j]);
		  }
		  
		  //std::cout << "2 " << std::endl;
		  costValue.index = good_sol_index;
		  costValue.cost = cost;
		  costs[i] = costValue;
		  //std::cout << std::endl;
	     	}
	     	
	     	//std::cout << "3 " << std::endl;
	     	int lowCostIndex = 0;
	     	
	     	for (int i=1; i < good_solutions.size(); i++)
	     	{
	     	  if (costs[i].cost < costs[lowCostIndex].cost)
	     	  {
	     	   lowCostIndex = i;
	     	  }
	     	}
	     	
	     	//std::cout << "4 " << std::endl;
	     	const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(costs[lowCostIndex].index);
	     	this_sol_free_params = (int)sol.GetFree().size(); 
		std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
		  
	     	sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
		//std::cout << "Selected Solution Index " << costs[lowCostIndex].index << std::endl;
	     	for( std::size_t j = 0; j < solvalues.size(); ++j)
	     	 {
	     	    target_pose[j] = solvalues[j];
		    //printf(" target Pos after selection %.15f, ", target_pose[j]);
		 }
		 
		 std::cout << std::endl;
     	}
     }
     return bSuccess;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_right_angle_srv");   
    ros::NodeHandle n;    
    client = n.serviceClient<pluto::arm_angles_msg>("robot_controller");


    string line, token, balanceString;
    IKREAL_TYPE eerot[9],eetrans[3], current_pose[6], target_pose[6];
    int delimPos = 0;
    int startPos = 0;

    ifstream pointsFile;
    pointsFile.open("/home/tuttu/ros_ws/src/pluto/src/pointsList.txt");
    if(!pointsFile.is_open()) 
    {
    	perror("Error open");
	exit(EXIT_FAILURE);
    }
    
    int lineCount = 0;
    char input;
    
    while(ros::ok())
    {
	   while(getline(pointsFile, line)) 
	   {
		   balanceString = line;
		   startPos = 0;
		   std::cout << lineCount++ << std::endl;
		     
	    	   for (int i=0; i < 9; i++)
		   {
			   delimPos = balanceString.find (',', startPos);
			   printf(" %dth Pos =%d ", i, delimPos);
			   token = balanceString.substr(startPos, delimPos - startPos);
			   std::cout << token;
			   eerot[i] = stof(token, NULL);
			    
			   startPos = delimPos + 1;
		   }
		     
		   for (int i=0; i<3; i++)
		   {
			   delimPos = balanceString.find (',', startPos);
			   printf(" %dth Pos =%d ", i, delimPos);
			   token = balanceString.substr(startPos, delimPos - startPos);
			   std::cout << token;
			   eetrans[i] = stof(token, NULL);      
			   startPos = delimPos + 1;
		   }
		     
		   findSolution(eerot,eetrans, current_pose, target_pose);
		   for (int i=0; i < 6; i++)
		   {
		   	current_pose[i] = target_pose[i];
		   }
		 
		    
		   srv.request.arm = 0; //0 is right arm, 1 is left arm
		   srv.request.torso_ang = target_pose[0];
		   srv.request.shoulder_ang = target_pose[1];
		   srv.request.upper_arm_bottom_ang = target_pose[2];
		   srv.request.upper_arm_top_ang = target_pose[3];
		   srv.request.forearm_bottom_ang = target_pose[4];
		   srv.request.forearm_top_ang = target_pose[5];
		   
		   cout << "Press any key for next arm motion" << endl;
		   cin >> input;
		   
		   cout << "calling client" << endl;
		   if(client.call(srv))
		   {
		          if(srv.response.pos_ok)
		          {
		          }
		          else
		          {
		          	cout << "something went wrong";
		          }
                  }
                  else
                  {
                  	ROS_ERROR("Failed to call service robot_controller");
                  }
    	   }
    }
    return 0;
}


 
