/****************************************************************\
 *	 		rcx version 0.02			*
 * 		The second version of RollcageX 		*
 * livence: gplv2 (or later, or whatever Nossta wants it to be)	*
 *								*
 * will probably only compile on dev friendly systems (like	*
 * linux, BSD, OSX), please post on the forum if you get it to	*
 * compile on others.						*
 *						/Slinger	*
\****************************************************************/

#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>

#include <ode/ode.h>
#include "drawstuff.h"

#define dsSetColour dsSetColor //just to annoy: use brittish words ;)

long long int timecount;
int stepwarncount = 0;

//include other parts of the code
#include "main.h"
#include "dynamics.c"
#include "graphics.c"
#include "loaders.c"

void start()
{
	printf ("Welcome to rcx 0.02\n"
		"use the 'a' 'd' 'w' and 's' keys for turning and driving\n"
		"spacebar resets controls\n");
	float posxyz[3] = {0, -15, 10}; //cam position
	float rotxyz[3] = {90, -30, 0};//cam rotation
	dsSetViewpoint (posxyz, rotxyz);
}

void command (int key)
{
	switch (key)
	{
		case 'w':
			control.throttle = 1;
		break;

		case 's':
			control.throttle = -0.8;
		break;

		//no breaks yet, because drawstuff limitations, we'll just use it for reset instead
		case ' ':
			control.throttle = 0;
			control.steer = 0;
		break;

		case 'a':
			control.steer = -car_max_steer;
		break;
	
		case 'd':
			control.steer = car_max_steer;
		break;
	}
}

void step (int pause)
{
	if (!pause)
	{
		simulate_race(internal.stepsize);
	}

	//do nothing until the time simulated has been wasted
	timecount += internal.stepsize*1000000;
	long long sleeptime;
	struct timeval tmpcount;

	gettimeofday (&tmpcount, NULL);
	sleeptime = timecount - (tmpcount.tv_sec*1000000 + tmpcount.tv_usec);

//	printf("> %i\n", sleeptime); // * uncomment this if you want to see how wastefull the loop was

	if (sleeptime < 0)
	{
		stepwarncount++; //we have no time to waste!
		printf("%i\n", stepwarncount);
	}

	else //good, we can sleep to get realistic syncing
	{
		usleep (sleeptime); //sleep to sync
//		gettimeofday (&tmpcount, NULL); // * uncomment this two to if you want to see how wastefull the loop was
//		printf(">> %i\n\n", (timecount - tmpcount.tv_sec*1000000 + tmpcount.tv_usec)/1000); // *
	}

	draw_race(); //draw the scene
}

int main (int argc, char **argv)
{
	if (load_car_data ("data/cars/Nemesis/Venom/config", &car[0])) return 1;
	if (load_internal_data ("data/internals/config", &internal)) return 1;

	dInitODE();

	dMatrix3 rotation; //temporary rotation variable
	int loop; //temporary loop variable
	dMass mass; //temporary mass variable
	const dReal *wpos; //temporary wheel position variable (pointer)

	//create a simple world (the track)
	track = dWorldCreate ();
	track_space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0); //needed for collisions
	dWorldSetGravity (track, 0, 0, -track_gravity);
	
	//add track objects
	track_object[0] = dCreatePlane (track_space, 0, 0, 1, 0); //ground
	
	track_object[1] = dCreateBox (track_space, 20, 14, 1);
	dRFromAxisAndAngle (rotation, 1, 0, 0, 0.3);
	dGeomSetPosition (track_object[1], 0, 30, 1.8);
	dGeomSetRotation (track_object[1], rotation);

	track_object[2] = dCreateBox (track_space, 20, 14, 1);
	dRFromAxisAndAngle (rotation, 1, 0, 0, 0.8);
	dGeomSetPosition (track_object[2], 0, 40, 8);
	dGeomSetRotation (track_object[2], rotation);

	track_object[3] = dCreateBox (track_space, 20, 60, 1);
	dRFromAxisAndAngle (rotation, 1, 0, 0, M_PI/2);
	dGeomSetPosition (track_object[3], 0, 70, 20);
	dGeomSetRotation (track_object[3], rotation);
	//create a simple car
	car_space = dSimpleSpaceCreate (track_space);

	//main body
	car_body = dBodyCreate (track);
	dMassSetBoxTotal (&mass, car[0].mass, car_width, car_lenght, car_height);
	dBodySetMass (car_body, &mass);
	car_geom = dCreateBox (car_space, car_width, car_lenght, car_height);
	dGeomSetBody (car_geom, car_body);
	dBodySetPosition (car_body, 0, 0, track_start_height);

	//wheels
	dRFromAxisAndAngle (rotation, 0, 1, 0, M_PI*0.5);
	for (loop=0; loop<4; loop++)
	{
		car_wheel_body[loop] = dBodyCreate (track);
		dBodySetRotation (car_wheel_body[loop], rotation);
		dMassSetCylinderTotal (&mass, wheel_mass, 3, internal.wheel_radius, internal.wheel_width);
		dBodySetMass (car_wheel_body[loop], &mass);
		dBodySetFiniteRotationMode (car_wheel_body[loop], 1);

		car_wheel_geom[loop] = dCreateCylinder (car_space, internal.wheel_radius, internal.wheel_width);
		dGeomSetBody (car_wheel_geom[loop], car_wheel_body[loop]);
	}

	//wheel position
	dBodySetPosition (car_wheel_body[0], wheel_right_position, wheel_front_position, track_start_height);	//front right wheel
	dBodySetPosition (car_wheel_body[1], wheel_left_position, wheel_front_position, track_start_height);	//front left wheel
	dBodySetPosition (car_wheel_body[2], wheel_right_position, wheel_rear_position, track_start_height);	//rear right wheel
	dBodySetPosition (car_wheel_body[3], wheel_left_position, wheel_rear_position, track_start_height);	//rear left wheel

	//wheel joints
	car_joint_group = dJointGroupCreate (0);
	for (loop=0; loop<4; loop++)
	{
		car_wheel_joint[loop] = dJointCreateHinge2 (track,car_joint_group);
		dJointAttach (car_wheel_joint[loop], car_body, car_wheel_body[loop]);
		wpos = dBodyGetPosition (car_wheel_body[loop]);
		dJointSetHinge2Anchor (car_wheel_joint[loop], wpos[0], wpos[1], wpos[2]);
		dJointSetHinge2Axis1 (car_wheel_joint[loop], 0, 0, 1);
		dJointSetHinge2Axis2 (car_wheel_joint[loop], 1, 0, 0);
	}
	
	//wheel suspension
	for (loop=0; loop<4; loop++)
	{
		dJointSetHinge2Param (car_wheel_joint[loop], dParamSuspensionERP, car[0].suspension_erp);
		dJointSetHinge2Param (car_wheel_joint[loop], dParamSuspensionCFM, car[0].suspension_cfm);
	}
	
	//prevent turning (steering) on rear wheels
	for (loop=2; loop<4; loop++)
	{
		dJointSetHinge2Param (car_wheel_joint[loop], dParamLoStop, 0);
		dJointSetHinge2Param (car_wheel_joint[loop], dParamHiStop, 0);
	}

	//drawstuff... stuff
	dsFunctions drawfunc;
	drawfunc.version = DS_VERSION;
	drawfunc.start = &start;
	drawfunc.step = &step;
	drawfunc.command = &command;
	drawfunc.stop = 0;
	drawfunc.path_to_textures = "drawstuff/textures";

	//get current time
	struct timeval tmpcount;
	gettimeofday (&tmpcount, NULL);
	timecount = (tmpcount.tv_sec*1000000 + tmpcount.tv_usec);
	timecount += 1000;

	//run race
	dsSimulationLoop (argc, argv, 640, 480, &drawfunc);

	//after race cleanup
	dJointGroupDestroy (car_joint_group);
	dSpaceDestroy (car_space);

	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (track_space);
	dWorldDestroy (track);
	dCloseODE();

	printf("\nstepsize-to-low warnings:%i\n", stepwarncount);
/*	printf ("\n%g\n", counter);
	printf ("\n%g\n", timecount);
	printf ("\n%g\n", time(NULL));
	printf ("\n%g\n", (time(NULL)-timecount));
	counter = time(NULL);
	counter -= time(NULL);
	printf ("\n%g\n", counter);
	printf ("\n%g\n", counter*2);
*/
	return 0;
}

