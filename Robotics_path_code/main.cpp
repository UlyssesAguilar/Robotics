#include <stdio.h>
#include <math.h>
#include "obstacles.h"

/* constants for the length over x and y.
 * Made the length be 17 by 11, however, the robot
 * will be able to move any tiles in the 16x10 course
 */
#define LENGTH_X 16
#define LENGTH_Y 10

/* constants for the maximum distance the robot can go
 * will be 16 tiles. Added the tile width that was given
 * by the professor as a constant
 */
#define TILE_WIDTH 0.305
#define MAX_DISTANCE 40

#define ROBOT_ROAMING_AREA 1
#define GRID_CORNER 2
#define GRID_EDGE 3

/* constants for all the possible directions that the
 * robot can travel
 */
#define UP 10
#define DOWN 20
#define RIGHT  30
#define LEFT 40


/*
 * STRUCTURES
 */

struct Point_State {
    int start;
    int goal;
    int pathMarker;
    int processed;
    int next_point;
    int obstacle;
};

struct Single_Point {
    struct Point_State pointState;
    struct Single_Point* move_up;
    struct Single_Point* move_down;
    struct Single_Point* move_left;
    struct Single_Point* move_right;
    int grid_value;
    int grid_type;
    int f_dist;
    int h_dist;
    int g_dist;
    int x;
    int y;
    int move;
    int pos;
};

struct Single_Point grid_field[LENGTH_Y][LENGTH_X];
struct Single_Point grid_path[LENGTH_X*LENGTH_Y];
struct Single_Point *goal_position;
struct Single_Point *start_position;
int grid_check = 1;
/*
 * Function Name: Init
 * Description: Initialize grid
 * Returns: int ret
 */
int initialize_points_in_grid() {
    int i,j,k;
    
    /* Initialize points */
    for (i = 0; i < LENGTH_Y; i++) {
        for (j = 0; j < LENGTH_X; j++) {
            grid_field[i][j].x = j;
            grid_field[i][j].y = i;
            grid_field[i][j].pointState.pathMarker = 0;
            grid_field[i][j].pointState.processed = 0;
            grid_field[i][j].pointState.next_point = 0;
            grid_field[i][j].pointState.obstacle = 0;
            grid_field[i][j].pointState.start = 0;
            grid_field[i][j].pointState.goal = 0;
            grid_field[i][j].grid_value = 1;
            grid_field[i][j].pos = 0;
            
            /* Conditionally set point types */
            
            grid_field[i][j].grid_type = ROBOT_ROAMING_AREA;
            grid_field[i][j].pointState.processed = 0;
            grid_field[i][j].pointState.next_point = 0;
        }
    }
    //printf("start point(%d,%d)", convert_to_feet(start[0]),)
    /* Set start and goal locations */
    for (i = 0; i < LENGTH_Y; i++) {
        for(j = 0; j < LENGTH_X; j++) {
            if ( (grid_field[i][j].x == convert_to_feet(start[0]) )
                && (grid_field[i][j].y == convert_to_feet(start[1])) ) {
                grid_field[i][j].pointState.start = 1;
                grid_field[i][j].pointState.processed = 0;
                grid_field[i][j].pointState.next_point = 0;
                grid_field[i][j].grid_value = 0;
                start_position = &grid_field[i][j];
            }
            else if ( (grid_field[i][j].x == convert_to_feet(goal[0]) )
                     && (grid_field[i][j].y == convert_to_feet(goal[1])) ) {
                grid_field[i][j].pointState.goal = 1;
                grid_field[i][j].pointState.processed = 0;
                grid_field[i][j].pointState.next_point = 0;
                grid_field[i][j].grid_value = 0;
                goal_position = &grid_field[i][j];
            }
        }
    }
    
    /* Set obstacle locations*/
    for (i = 0; i < LENGTH_Y; i++) {
        for(j = 0; j < LENGTH_X; j++) {
            for (k = 0; k < MAX_OBSTACLES; k++) {
                if ( (grid_field[i][j].x == convert_to_feet(obstacle[k][0]) )
                    && (grid_field[i][j].y == convert_to_feet(obstacle[k][1]) ) ) {
                    grid_field[i][j].pointState.obstacle = 1;
                    grid_field[i][j].pointState.processed = 0;
                    grid_field[i][j].pointState.next_point = 0;
                    grid_field[i][j].grid_value = -88;
                }
            }
        }
    }
    // set possible movements for each block 
    for (i = 0; i < LENGTH_Y; i++) 
    {   
        for (j= 0; j < LENGTH_X; j++) 
        {    
            /* Connect adjacent neighbors */
            if(i+1<LENGTH_Y)  //going up
            {   
                if(grid_field[i+1][j].grid_value == -88)
                {
                    grid_field[i][j].move_up = &grid_field[i+1][j];
                }
                else
                {
                    //printf("(%d,%d) up possible\n",j,i);
                    grid_field[i][j].move_up = &grid_field[i+1][j];
                    grid_field[i][j].move_up->pos = 1;
                }
            }
            else
            {   //printf("(%d,%d) up not possible\n",j,i);
                //grid_field[j][i].move_up->pos = 0;
            }
            if(i-1>=0)  // going down 
            {
                if(grid_field[i-1][j].grid_value ==-88)
                {   
                    grid_field[i][j].move_down = &grid_field[i-1][j];
                }
                else
                {   //printf("(%d,%d) down possible\n",j,i);
                    grid_field[i][j].move_down = &grid_field[i-1][j];
                    grid_field[i][j].move_down->pos = 1;
                    //printf("(%d,%d) down possible (%d,%d) pos = %d \n",j,i,j,i-1,grid_field[j][i].move_down->pos);
                }
            }
            else
            {   //printf("(%d,%d) down not possible\n",j,i);
                //grid_field[j][i].move_down->pos = 0;
                //printf("past grid\n");
            }
            if(j-1>=0)  //moving right 
            {   //printf("past left condition\n");
                if(grid_field[i][j-1].grid_value ==-88)
                {   
                    grid_field[i][j].move_right = &grid_field[i][j-1];
                    
                }
                else
                {
                    //printf("(%d,%d) left possible\n",j,i);
                    grid_field[i][j].move_right = &grid_field[i][j-1];
                    grid_field[i][j].move_right->pos = 1;
                }
            }
            else
            {
                //printf("(%d,%d) left not possible\n",j,i);
                //grid_field[j][i].move_left->pos = 0;
            }
            if(j+1<LENGTH_X) //left
            {
                if(grid_field[i][j+1].grid_value == -88)
                {   
                    grid_field[i][j].move_left = &grid_field[i][j+1];
                }
                else
                {   //printf("(%d,%d) right possible\n",j,i);
                    grid_field[i][j].move_left = &grid_field[i][j+1];
                    grid_field[i][j].move_left->pos = 1;
                }
            }
            else
            {   //printf("(%d,%d) right not possible\n",j,i);
                //grid_field[j][i].move_right->pos = 0;
            }
            
            
            
        }
    }
    
    return 1;
}

bool check_for_deadend(struct Single_Point current, int dir)
{   struct Single_Point cur = current;
    printf("current node in deadend check (%d,%d)\n", cur.x,cur.y);
    if(dir == UP)
    {
        if(cur.x == goal_position->x && goal_position -> y >= cur.y)
        {
            return false;
        }
        if(cur.x < goal_position->x)
        {   printf("in if statment\n");
            while(cur.y < LENGTH_Y-1)
            {   printf("in loop\n");
                if(cur.move_left->pos)
                {   
                    return false;
                }
                else
                {
                    cur = *cur.move_up;
                }
            }
            return true;
        }
         if(cur.x > goal_position->x)
        {   while(cur.y < LENGTH_Y-1)
            {
                if(cur.move_right->pos)
                {   
                    return false;
                }
                else
                {
                    cur = *cur.move_up;
                }
            }
            return true;
        }
        
    }
    if(dir == DOWN)
    {
        if(cur.x == goal_position->x && goal_position -> y <= cur.y)
        {
            return false;
        }
        if(cur.x < goal_position->x)
        {   while(cur.y >-1)
            {
                if(cur.move_left->pos)
                {   
                    return false;
                }
                else
                {
                    cur = *cur.move_down;
                }
            }
            return true;
        }
        if(cur.x > goal_position->x)
        {   while(cur.y >-1)
            {
                if(cur.move_right->pos)
                {   
                    return false;
                }
                else
                {
                    cur = *cur.move_down;
                }
            }
            return true;
        }
    }
    if(dir == LEFT)
    {
        if(cur.y == goal_position->y && cur.x <= goal_position->x)
        {
            return false;
        }
        if(cur.y > goal_position->y)
        {
            while( cur.x < LENGTH_X-1)
            {
                if(cur.move_down->pos)
                {
                    return false;
                }
                else
                {
                    cur = *cur.move_left;
                }
                
            }
            return true;
        }
        if(cur.y < goal_position->y)
        {
            while( cur.x < LENGTH_X-1)
            {
                if(cur.move_up->pos)
                {
                    return false;
                }
                else
                {
                    cur = *cur.move_left;
                }
                
            }
            return true;
        }
    }
    if(dir == RIGHT)
    {
        if(cur.y == goal_position->y && cur.x >= goal_position->x)
        {
            return false;
        }
        if(cur.y > goal_position->y)
        {
            while( cur.x > -1)
            {
                if(cur.move_down->pos)
                {
                    return false;
                }
                else
                {
                    cur = *cur.move_right;
                }
                
            }
            return true;
        }
        if(cur.y < goal_position->y)
        {
            while( cur.x >-1)
            {
                if(cur.move_up->pos)
                {
                    return false;
                }
                else
                {
                    cur = *cur.move_right;
                }
                
            }
            return true;
        }
    }
    return false;

}
int manhattan_distance_goal(int x, int y) {
    
    int dist = 0;
    dist = abs((x-goal_position->x));
    dist = dist + abs((y-goal_position->y));
    return dist;
}
int manhattan_distance_start(int x, int y) {
    
    int dist = 0;
    dist = abs(((int)start_position->x-x)+(start_position->y-y));

    return dist;
}


void print_grid_field(void) {
    int i,j;
    
    printf("VALUES:\n");
    for (i = 0; i < LENGTH_Y; i++) {
        for (j = 0; j < LENGTH_X; j++) {
            if(grid_check){
                printf(" %d ", grid_field[i][j].pos);
            }
        }
        printf("\n");
        printf("\n");
    }
}


int find_best_path(void) {
    int i = 0;
    int point_count = 0;
    int curh = 0;
    struct Single_Point current = *start_position;

    
    /* Start from start position  */
    do 
    {   //variables to determine which way is best
        int up=0;
        int down=0;
        int left= 0;
        int right= 0;
        printf("current (%d,%d) goal position (%d,%d) \n",current.x,current.y,goal_position->x,goal_position->y);
        //calculate each direction g, h, and f values 
        if(current.y != (LENGTH_Y -1))
        {
            if(current.move_up->pos)
            {   //printf("calculating h and g\n");
                //printf("possible check for up = %d\n",current.move_up->pos);
                current.move_up->h_dist = curh+1;
                current.move_up->g_dist = manhattan_distance_goal(current.move_up->x,current.move_up->y);
                current.move_up->f_dist = current.move_up->h_dist + current.move_up->g_dist;
                up = current.move_up->f_dist;
                printf("up (%d,%d) h = %d  g = %d f = %d \n",current.move_up->x,current.move_up->y,current.move_up->h_dist,current.move_up->g_dist,current.move_up->f_dist);
                if(check_for_deadend(current, UP))
                {
                    up = 1000;
                    printf("up is dead end");
                }
                
                
            }
            else
            {
                up = 1000;
            }
        } 
        else
        {
            up = 1000;
        }
        //printf("current node y = %d\n",current.y);
        if(current.y != 0)
        {
            if(current.move_down->pos)
            {  
                current.move_down->h_dist = curh+1;
                current.move_down->g_dist = manhattan_distance_goal(current.move_down->x,current.move_down->y);
                current.move_down->f_dist = current.move_down->h_dist + current.move_down->g_dist;
                down = current.move_down->f_dist;
                printf("down(%d,%d) h = %d  g = %d f = %d \n",current.move_down->x,current.move_down->y,current.move_down->h_dist,current.move_down->g_dist,current.move_down->f_dist);
                if(check_for_deadend(current, DOWN))
                {
                    down = 1000;
                }
            }
            else
            {   //printf("down not possible\n");
                down = 1000;
            }
        }
        else
        {
            down =1000;
        }
        
        
        if(current.x !=0) 
        {
            if(current.move_right->pos)
            {
                current.move_right->h_dist = curh+1;
                current.move_right->g_dist = manhattan_distance_goal(current.move_right->x,current.move_right->y);
                current.move_right->f_dist = current.move_right->h_dist + current.move_right->g_dist;
                right = current.move_right->f_dist;
                printf("right (%d,%d) h = %d  g = %d f = %d \n",current.move_right->x,current.move_right->y,current.move_right->h_dist,current.move_right->g_dist,current.move_right->f_dist);
                if(check_for_deadend(current, RIGHT))
                {
                    right = 1000;
                }
            }
            else
            {
                right = 1000;
            }
        }
        else
        {
            right = 1000;
        }
        if( current.x !=LENGTH_X-1)
        {   //printf("moveleft possible check = %d",current.move_left->grid_value);
            if(current.move_left->pos)
            {
                current.move_left->h_dist = curh+1;
                current.move_left->g_dist = manhattan_distance_goal(current.move_left->x,current.move_left->y);
                current.move_left->f_dist = current.move_left->h_dist + current.move_left->g_dist;
                left = current.move_left->f_dist;
                printf("left (%d,%d) h = %d  g = %d f = %d \n",current.move_left->x,current.move_left->y,current.move_left->h_dist,current.move_left->g_dist,current.move_left->f_dist);
                if(check_for_deadend(current, LEFT))
                {
                    left = 1000;
                }
            }
            else
            {
                left = 1000;
            }
        }
        else
        {
            left = 1000;
        }
        curh++;
        
        // pick possible direction with least f cost
        printf("f values calculated for all nighbor nodes\n");
        
        if(i > 0)  //make sureit doesnt back track
        {
            if(grid_path[i-1].move== UP)
            {
                down =1000;
            }
            else if(grid_path[i-1].move == LEFT)
            {
                right =1000;
            }
            else if(grid_path[i-1].move == RIGHT)
            {
                left=1000;
            }
            else if(grid_path[i-1].move == DOWN)
            {
                up = 1000;
            }
        }
        if( up <= down && up <= right && up<=left)
        {   
            grid_path[i] = current;
            current = *current.move_up;
            grid_path[i].move = UP;
            printf("up\n");
        }
        else if(down <= up && down <= right && down <= left)
        {
            grid_path[i] = current;
            current = *current.move_down;
            grid_path[i].move = DOWN;
            printf("down\n");
        }
        else if(left <= up && left <= down && left <= right)
        {
            grid_path[i] = current;
            current = *current.move_left;
            grid_path[i].move = LEFT;
            printf("left\n");            
        }
        else
        {
            grid_path[i] = current;
            current = *current.move_right;
            grid_path[i].move = RIGHT;
            printf("right\n");
        } 
        printf("after current (%d,%d) goal position (%d,%d) \n",current.x,current.y,goal_position->x,goal_position->y);
        point_count++;
        
        grid_path[i].pointState.processed = 1;
        
        i++;
        
    }
    while(current.x != goal_position->x || current.y != goal_position->y);
    printf("path found \n");
    /* save end postion */
    grid_path[i] = current;
    point_count++;
    
    if (grid_check) {
        printf("PATH: \n");
        for (i = 0; i < point_count; i++) {
            printf("(%d, %d): %d -> %d\n",grid_path[i].x, grid_path[i].y, grid_path[i].grid_value, grid_path[i].move);
            grid_path[i].grid_value = 10+i;
        }
        printf("point count = %d\n", point_count);
    }
    
    return 1;
}

int convert_to_feet(double m) {
    int ret = 0;
    double iptr;
    double in;
    
    m = m / TILE_WIDTH;
    
    /* Store integral part of m */
    in = modf(m, &iptr);
    
    if (in >= 0.5)
        m++;
    
    ret = (int) m;
    
    return ret;
}
void print_ob()
{   int i,j;
    printf("obsticles: \n");
    for(i=0; i<MAX_OBSTACLES; i++) {
       for(j=0;j<2;j++) {
           printf("%d ", convert_to_feet(obstacle[i][j]));
           if(j==1){
               printf("\n");
            }
        }
    }

}


/* Main routine */
int main() {
    initialize_points_in_grid();
    
    find_best_path();
    print_grid_field();
    //print_ob();

}
