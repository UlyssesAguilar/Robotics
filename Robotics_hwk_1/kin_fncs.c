#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

void mat_mult(double a[4][4],double b[4][4], double res[4][4]) 
{   
   int r1=4, c1=4, r2=4, c2=4, i, j, k;
   /*
   printf("first matrix: \n");
   for(i=0; i<4; i++) {
      for(j=0;j<4;j++) {
         printf("%f ", a[i][j]);
         if(j==3){
            printf("\n");
         }
      }
   }
   printf("second matrix: \n");
   for(i=0; i<4; i++) {
      for(j=0;j<4;j++) {
         printf("%f ", b[i][j]);
         if(j==3){
            printf("\n");
         }
      }
   }
   */
   if (c1 != r2) 
   {
      printf("matrix not the right size");
   } 
   else 
   {  
      for(i=0; i<r1; ++i)
         for(j=0; j<c2; ++j) 
         {
            res[i][j] = 0;
         }
      for(i=0; i<r1; ++i)
         for(j=0; j<c2; ++j)
            for(k=0; k<c1; ++k)
            {
               res[i][j]+=a[i][k]*b[k][j];
            }
      /*
      printf("Result matrix: \n");
      for(i=0; i<4; i++) {
         for(j=0;j<4;j++) {
            printf("%f ", res[i][j]);
            if(j==3){
               printf("\n");
            }
         }
      }
      */
      
   }
}

fwd_kin(theta, x)
double *theta;
double x[3];
{
   //printf("t0 = %f  t1 = %f   t2 = %f     t3 = %f \n",theta[0],theta[1],theta[2],theta[3]);
   double h0_1[4][4] = {{cos(theta[0]),       0        , sin(theta[0]),(-0.05*cos(theta[0]))},
                        {sin(theta[0]),       0        ,-cos(theta[0]),(-0.05*sin(theta[0]))}, 
                        {     0       ,       1        ,        0     ,         0.35     },
                        {     0       ,       0        ,        0     ,          1        } };

   double h1_2[4][4] = { {cos(theta[1]),sin(theta[1]),    0   ,(0.5*cos(theta[1]))},
                         {-sin(theta[1]),cos(theta[1]) ,    0   ,(-0.5*sin(theta[1]))}, 
                         {      0      ,      0       ,    1   ,       0.05        },
                         {      0      ,      0       ,    0   ,        1          } };

   double h2_3[4][4] = { {cos(theta[2]),sin(theta[2]),    0    ,(0.35*cos(theta[2]))},
                         {-sin(theta[2]),cos(theta[2]) ,    0    ,(-0.35*sin(theta[2]))}, 
                         {     0       ,      0       ,    1    ,      -0.05         },
                         {     0       ,      0       ,    0    ,        1           } };

   double h3_4[4][4] = { {cos(theta[3]),sin(theta[3]),   0   ,(0.25*cos(theta[3]))}, 
                         {-sin(theta[3]),cos(theta[3]) ,   0   ,(-0.25*sin(theta[3]))}, 
                         {      0      ,      0       ,   1   ,       0.05         },
                         {      0      ,      0       ,   0   ,         1          } };
   double res1[4][4];
   double res2[4][4];
   double res3[4][4];
   mat_mult( h2_3, h3_4, res1);
   mat_mult( h1_2, res1, res2);
   mat_mult( h0_1, res2, res3);
   x[0] = res3[0][3];
   x[1] = res3[1][3];
   x[2] = res3[2][3];

   //printf("x = %f\n",x[0]);
   //printf("y = %f\n",x[1]);
   //printf("z = %f\n",x[2]);
}


inv_kin(x, theta)
double *x;
double theta[6];
{
   theta[4] = 0.00000000;
   double t = atan2(x[1],x[0]);
   double r = sqrt((x[0]*x[0])+(x[1]*x[1]));
   double r1 = sqrt((r*r)-(0.05*0.05));

   double beta = asin(.05/r);
   double r2 = cos(beta)*r;
   theta[0] = t + beta;
   double h = x[2] + 0.25;
   double d  = 0.05;

   double b = r2 +d;
    
   double a = h-0.35;
   
   double rd  = sqrt((a*a)+(b*b));
   
   double ph1= atan2(a,b);
   double temp1 = ((0.35*0.35)-(rd*rd)-(0.5*0.5))/(-2*0.5*rd);
   double ph2 = acos(temp1);
   
   theta[1] = -ph1 -ph2;
   
   double temp2 = ((rd*rd)-(0.5*0.5)-(0.35*0.35))/(-2*0.5*0.35);
   double ph3 =  acos(temp2);
   
   if (ph3 == 0)
   {
      theta[2] = ph3;
   }
   else
   {
      theta[2] = M_PI-ph3;
   }
   theta[3] = M_PI/2 -theta[1]-theta[2];

}

