//============================================================================
// Name        : Final Project
// Author      : Ulysses Aguilar
// Version     :
// Copyright   : $(copyright)
// Description : soccer playing robot
//============================================================================

#include <ev3.h>
#include <string>
#include <math.h>
#include <iostream>

#define RED 5;
#define GREEN 3;
#define FULL 360;
#define UP 0;
#define DOWN 1;
#define LEFT 2;
#define RIGHT 3;

int scan_deg=0;
int full_turn = 680;   //degrees to make a 360 degree turn
int ball_reading = 0;
int found_ball=4;
int hit_bound = 0;
int opp_color = GREEN;
int count = 0;
int orient = UP;
int zero = 0; //initial orientation

void wander();
int check_color()
{
	int color = readSensor(IN_3);
	if(color == opp_color)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void get_orient()
{
	int gyro;
	gyro = readSensor(IN_2);
	if(gyro>=0)
	{
		if(zero>=0)
		{
			gyro = gyro-zero;
		}
		else
		{
			gyro = gyro + abs(zero);
		}
	}
	else  //gyro is negative
	{
		if(zero>=0)
		{
			gyro = gyro-zero;
		}
		else
		{
			gyro = gyro + abs(zero);
		}
	}

	while(abs(gyro)>360)
	{
		if(gyro>=0)
		{
			gyro=gyro-360;
		}
		else
		{
			gyro= gyro+360;
		}
	}
	if((gyro>=-45 && gyro<=45) || (gyro<=-315 && gyro>=-360)|| (gyro>=315 && gyro<=360))
	{
		orient=UP;
	}
	else if((gyro>=-135 && gyro<=-45)||  (gyro>=225 && gyro<=315))
	{
		orient = LEFT;
	}
	else if((gyro<=135 && gyro>= 45)||  (gyro<=-225 && gyro>=-315))
	{
		orient = RIGHT;
	}
	else
	{
		orient = DOWN;
	}
}
void cw90()
{
	int left_deg = 0;
		left_deg = MotorRotationCount(OUT_A);
		while(abs(left_deg) < 185)
		{
			Wait(100);
			//Turn on ball,left, and right sensors
			left_deg = MotorRotationCount(OUT_A);
			OnFwdReg(OUT_A, 100);
			OnFwdReg(OUT_D, -100);
		}
		Off(OUT_AD);
		ResetRotationCount(OUT_AD);
}
void ccw90()
{
	int left_deg = 0;
	left_deg = MotorRotationCount(OUT_A);
	while(abs(left_deg) < 185)
	{
		Wait(100);
		//Turn on ball,left, and right sensors
		left_deg = MotorRotationCount(OUT_A);
		OnFwdReg(OUT_D, 100);
		OnFwdReg(OUT_A, -100);
	}
	Off(OUT_AD);
	ResetRotationCount(OUT_AD);
}


void forward()
{	int left_deg = 0;
	while(abs(left_deg)<500)
	{
		Wait(100);
		left_deg = MotorRotationCount(OUT_A);

		OnFwdSync(OUT_AD, 90);
		if(check_color())
		{
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			RotateMotor(OUT_AD, -90,624);
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			cw90();
			wander();
		}
	}

	ResetRotationCount(OUT_AD);
	Off(OUT_AD);
}


void forward_short()
{	int left_deg = 0;
	while(abs(left_deg)<500)
	{
		Wait(100);
		left_deg = MotorRotationCount(OUT_A);

		OnFwdSync(OUT_AD, 90);
		if(check_color())
		{
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			RotateMotor(OUT_AD, -90,624);
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			wander();
		}
	}

	ResetRotationCount(OUT_AD);
	Off(OUT_AD);
}

void wander()
{
	int ball_sensor = 0;
	int left_deg=0;
	int ball_flag=0;
	int right_deg=0;
	Off(OUT_AD);
	ResetRotationCount(OUT_AD);

	//while less than one turn
	while(abs(left_deg) < full_turn)
	{
		Wait(100);
		//Turn on ball,left, and right sensors
		ball_sensor = readSensor(IN_1);
		right_deg = MotorRotationCount(OUT_D);
		left_deg = MotorRotationCount(OUT_A);
		if(check_color())
		{
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			RotateMotor(OUT_AD, 90,624);
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			wander();
		}
		//while looking for the ball value(0<4)
		if(ball_sensor<found_ball)
		{
			OnFwdReg(OUT_A, 30);
			OnFwdReg(OUT_D, -30);
		}
		else
		{
			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			ball_flag=1;
			break;
		}
	}
	Off(OUT_AD);
	ResetRotationCount(OUT_AD);
	right_deg = MotorRotationCount(OUT_A);
	left_deg = MotorRotationCount(OUT_D);
	if(ButtonIsDown(BTNCENTER))
	{
		exit(0);
	}
	//LcdPrintf(1,"degree %d\n", scan_deg);

	//Go to ball_sensor
	if(ball_flag==1)
	{
		//move forward if in front, ball_reading>4
		while(ball_sensor>found_ball)
		{
			get_orient();
			if(orient==0)
			{
				LcdPrintf(1,"UP\n");
				forward();
				ball_sensor = readSensor(IN_1);
			}
			else if(orient==3)
			{
				LcdPrintf(1,"RIGHT\n");
				forward_short();
				ccw90();
				cw90();
				ball_sensor = readSensor(IN_1);
			}
			else if(orient==2)
			{
				LcdPrintf(1,"LEFT\n");
				forward_short();
				cw90();
				ball_sensor = readSensor(IN_1);
			}
			else
			{
				LcdPrintf(1,"DOWN\n");
				Off(OUT_AD);
				ResetRotationCount(OUT_AD);
				RotateMotor(OUT_A, 90,20);
				forward();
				forward();
				wander();
			}
		}
	}
	else
	{
		if(count==0)
		{
			forward();
			if(ButtonIsDown(BTNCENTER))
			{
        		exit(0);
			}
			forward();
			count++;
		}
		if(count==1)
		{   Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			right_deg = MotorRotationCount(OUT_A);
			left_deg = MotorRotationCount(OUT_D);
			while(abs(left_deg) < 185)
				{

					Wait(100);
					//Turn on ball,left, and right sensors
					ball_sensor = readSensor(IN_1);
					right_deg = MotorRotationCount(OUT_D);
					left_deg = MotorRotationCount(OUT_A);
					if(ButtonIsDown(BTNCENTER))
					{
						exit(0);
					}

					//while looking for the ball value(0<4)
					if(ball_sensor<found_ball)
					{
						OnFwdReg(OUT_A, 30);
						OnFwdReg(OUT_D, -30);
					}
					//LcdPrintf(1,"left_degree:%d\n",left_deg);\

					else
					{
						count=0;
						wander();
					}
				}
			if(ButtonIsDown(BTNCENTER))
			        	{
			        		exit(0);
			  			}
			forward();
			if(ButtonIsDown(BTNCENTER))
			        	{
			        		exit(0);
			  			}
			forward();
			count++;
		}
		if(count==2)
		{

			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			right_deg = MotorRotationCount(OUT_A);
			left_deg = MotorRotationCount(OUT_D);
			while(abs(left_deg) < 370)
			{
				Wait(100);
				//Turn on ball,left, and right sensors
				ball_sensor = readSensor(IN_1);
				right_deg = MotorRotationCount(OUT_D);
				left_deg = MotorRotationCount(OUT_A);
				if(ButtonIsDown(BTNCENTER))
				{
	        		exit(0);
				}

				//while looking for the ball value(0<4)
				if(ball_sensor<found_ball)
				{
					OnFwdReg(OUT_D, 30);
					OnFwdReg(OUT_A, -30);
				}
								//LcdPrintf(1,"left_degree:%d\n",left_deg);\

				else
				{
					count=0;
					wander();
				}
			}
			if(ButtonIsDown(BTNCENTER))
			{
        		exit(0);
			}
			forward();
			forward();
			count++;
		}
        if(count==3)
        {
        	if(ButtonIsDown(BTNCENTER))
        	{
        		exit(0);
  			}
        	forward();
        	forward();
        	count++;
        }
		if(count==4)
		{

			Off(OUT_AD);
			ResetRotationCount(OUT_AD);
			right_deg = MotorRotationCount(OUT_A);
			left_deg = MotorRotationCount(OUT_D);
			while(abs(left_deg) < 370)
			{
				Wait(100);
	 			//Turn on ball,left, and right sensors
				ball_sensor = readSensor(IN_1);
				right_deg = MotorRotationCount(OUT_D);
				left_deg = MotorRotationCount(OUT_A);
				if(ButtonIsDown(BTNCENTER))
				{
					exit(0);
				}

				//while looking for the ball value(0<4)
				if(ball_sensor < found_ball)
				{
					OnFwdReg(OUT_D, 30);
					OnFwdReg(OUT_A, -30);
				}
				//LcdPrintf(1,"left_degree:%d\n",left_deg);
				else
				{
					count=0;
					wander();
				}
			}
			forward();
			forward();
			count =0;
		}
		if(ButtonIsDown(BTNCENTER))
		{
			exit(0);
		}

		wander();

		
	}
}


int main()
{
  InitEV3();
  LcdPrintf(1,"start3\n");
  setAllSensorMode(NXT_IR_SEEKER,GYRO_ANG,COL_COLOR,COL_COLOR);
  zero = readSensor(IN_2);
  while(ButtonIsUp(BTNCENTER))
  {
	  wander();
  }

  FreeEV3();
}
