#include <stdio.h>
#include <math.h>
double count = 0;
double av = 0;
double PD_control(theta, theta_dot, theta_ref, theta_dot_ref)
double theta, theta_dot, theta_ref, theta_dot_ref;
{
  count =count +.001;
  av -= av / 1000;
  av += theta_dot /1000;
  printf("torque =  %f  theta = %f, theta_dot = %f,   avg vel = %f  theta_ref = %f, theta_dot_ref = %f\n",count, theta, theta_dot,av, theta_ref, theta_dot_ref);
  
  double ret;
  if(count >= 9000) 
  {
    ret = 0.5;
  }
  else
  {
    ret= 0;
  }
  return(count);
  
  
  
}
