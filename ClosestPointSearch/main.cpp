#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include "CPS_AABBTree.h"

int main()
{
	std::fstream fs;
	fs.open("D:/MyData/Research/Data_Models/SurfaceMesh/PumpkinMesh.obj", std::fstream::in);
	// old version include test on both CGAL AABB tree and mine.
	ClosestPointSearch::AABBTree  tree();
}	
