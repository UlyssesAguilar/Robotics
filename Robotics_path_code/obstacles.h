
//
//  Header.h
//  Robotics_Porject_main
//
//  Created by Christian Diaz on 10/19/19.
//  Copyright © 2019 Christian Diaz. All rights reserved.
//

#ifndef obstacles_h
#define obstacles_h

#define MAX_OBSTACLES 25 /* maximum number of obstacles */

int num_obstacles = 13; /* number of obstacles */

// obstacle locations
double obstacle[MAX_OBSTACLES][2] =
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
    {1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
    {2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
    {3.353, 2.743},
    {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {-1,-1},{-1,-1},{-1,-1}};
//(2,9)(3,9)(4,9)(6,4)(6,5)(6,6)(6,7)

/*
double obstacle[MAX_OBSTACLES][2] = {
    {0.915, 0.305}, //(3,1)
    {0.915, 0.61}, //(3,2)
    {0.915, 0.915}, //(3,3)
    {0.915, 1.219}, //(3,4)
    {0.915, 1.524}, //(3,5)
    {2.135, 1.219}, //(7,4)
    {2.135, 1.524}, //(7,5)
    {2.135, 1.829}, //(7,6)
    {2.135, 2.134}, //(7,7)
    {2.135, 2.439}, //(7,8)
    {2.135, 2.744}, //(7,9)
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1},
    {-1,-1}
};
*/
double start[2] = {0.305, 1.219}; /* start location */
//double start[2] = {0.305, 0.61}; //(1,2)
double goal[2] = {3.658, 1.829}; /* goal location */
//double goal[2] = {3.05, 2.135}; //(10,7)
int initialize_points_in_grid(void);
int manhattan_path_distance(int block);
void print_grid_field(void);
int convert_to_feet(double m);
int find_best_path(void);


#endif /* obstacles_h */
