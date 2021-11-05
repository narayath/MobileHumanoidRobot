#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

#include <string.h>
#include <stdio.h> 
#include <iostream>
#include <fstream>

#include "ikfast61_plutoLeftArm.cpp"

#define IKREAL_TYPE IkReal 
const double pi=3.14159;

bool findSolution(IKREAL_TYPE eerot[9],IKREAL_TYPE eetrans[3], IKREAL_TYPE current_pose[6], IKREAL_TYPE target_pose[6])
{ 
     struct path_cost{
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
          
          if (solvalues[0] <= pi/2 && solvalues[0] >= -pi/2 && solvalues[1] >= 0 && solvalues[2] >= 0 && solvalues[4] >= 0)
	  {
	    good_solutions.push_back(i);
	  }
         /*
          for( std::size_t j = 0; j < solvalues.size(); ++j)
                    printf("%.15f, ", solvalues[j]);
          std::cout << std::endl;
          */
     	}
     }
     return bSuccess;
}


int main(int argc, char** argv)
{

    std::string line, token, balanceString;
    IKREAL_TYPE eerot[9],eetrans[3], current_pose[6], target_pose[6];
    int delimPos = 0;
    int startPos = 0;

    std::ifstream pointsFile;
    pointsFile.open("/home/tuttu/ros_ws/src/pluto/src/pointsList.txt");

   if(!pointsFile.is_open()) {
      perror("Error open");
      exit(EXIT_FAILURE);
   }
    int lineCount = 0;
    while(getline(pointsFile, line)) {
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
     std::cout << std::endl;
     for (int i=0; i < 9; i++)
     {
       std::cout << eerot[i] << ", ";
     }
     
     for (int i=0; i < 3; i++)
     {
       std::cout << eetrans[i] << ", ";
     }
     
     findSolution(eerot,eetrans, current_pose, target_pose);
     for (int i=0; i < 6; i++)
     {
      current_pose[i] = target_pose[i];
     }
     //std::cout << std::endl << line << std::endl;
    }
}


