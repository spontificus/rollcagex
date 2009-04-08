/****************************************************************\
 *	 		rcx version 0.01			*
 * 		somewhat a proof of concept of nothing 		*
 * livence: gplv2 (or later, or whatever Nossta wants it to be)	*
 *								*
 * will probably only compile on dev friendly systems (like	*
 * linux, BSD, OSX), please post on the forum if you get it to	*
 * compile on others.						*
 *								*
 * this is far from what this game will be, but at least it's	*
 * something (it's not even meant as a public release)		*
 *						/Slinger	*
\****************************************************************/

#include <ode/ode.h>
#include "drawstuff.h"

#define dsSetColour dsSetColor //just to annoy: use brittish words ;)

#define track_start_height 3
#define track_mu dInfinity
#define track_slip1 0.1
#define track_slip2 0.1
#define track_erp 0.9
#define track_cfm 0.001
#define track_gravity 9.82

#define track_objects 4 //_including_ the ground plane

//todo: add this data to a file and create a loader function for that file (and a file for the track data above)
#define car_lenght   9
#define car_width    6
#define car_height   2
#define car_mass     40
#define car_max_torque 500
#define car_suspension_erp 0.9
#define car_suspension_cfm 0.05
//and possibly a 3:rd file for this wheel data:
#define wheel_radius 2
#define wheel_width  2.5
#define wheel_mass   5
#define wheel_slip   0.05

//maybe not this data
#define wheel_front_position 3.5
#define wheel_rear_position -3.5
#define wheel_right_position 4.75
#define wheel_left_position -4.75
#define car_max_turn 0.6
#define contactpoints 8

dWorldID track;
dSpaceID track_space;
dGeomID  track_object[track_objects];
dJointGroupID contactgroup;

dSpaceID car_space; //have a space for car parts, never call dSpaceCollide with this space, since not needed
dBodyID  car_body;
dGeomID  car_geom;
dBodyID  car_wheel_body[4];
dGeomID  car_wheel_geom[4];
dJointID car_wheel_joint[4];
dJointGroupID car_joint_group;

dReal throttle=0, steer=0, direction=1;

void collisioncallback (void *data, dGeomID o1, dGeomID o2)
{
	if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) dSpaceCollide2 (o1, o2, data, &collisioncallback); //collide spaces (car) with other stuff
	else
	{
		dContact contact[contactpoints];
		int cont_count = dCollide (o1, o2, contactpoints, &contact[0].geom, sizeof(dContact)); //get contact points if normal objects

		if (cont_count){

		//the default track surface parameters (will be changed if neded)
		dSurfaceParameters default_surface;
		default_surface.mode = dContactApprox1 | dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM;
		default_surface.mu    = track_mu;
		default_surface.slip1 = track_slip1;
		default_surface.slip2 = track_slip2;
		default_surface.soft_erp = track_erp;
		default_surface.soft_cfm = track_cfm;

		//space id of geom container
		dSpaceID o1space = dGeomGetSpace (o1);
		dSpaceID o2space = dGeomGetSpace (o2);

		//TODO: add a loop to check the space ID:s against a list of car space ID:s (but there's only one car for the moment)
		if (o1space == car_space || o2space == car_space)
		{
			//if it's a wheel (== not the body), modify the surface parameters to make it more realistic
			if (o1 != car_geom && o2 != car_geom)
			{
				default_surface.slip1 = wheel_slip;
				default_surface.slip2 = wheel_slip;
			}

			//check the relative (to the car body) collision coordinates (just the height)
			dVector3 relpos;
			dBodyGetPosRelPoint (car_body, contact[0].geom.pos[0], contact[0].geom.pos[1],
					contact[0].geom.pos[2], relpos);

			if (relpos[2] < 0) direction = 1;
			if (relpos[2] > 0) direction = -1;
		}

		int loop;
		for (loop=0; loop<cont_count; loop++)
		{
			contact[loop].surface = default_surface;
			dJointAttach (dJointCreateContact (track, contactgroup, &contact[loop]),
				       dGeomGetBody (contact[loop].geom.g1),
				       dGeomGetBody (contact[loop].geom.g2));
		}

		}
	}
}

void start()
{
	printf ("Welcome to rcx 0.01\n"
		"it's all simple, use the 'a' and 'd' buttons for turning\n"
		"use 'w' for engine (forvard) at max, 's' for reverse engine at max, spacebar turns of the engine\n");
	float posxyz[3] = {0, -15, 10}; //cam position
	float rotxyz[3] = {90, -30, 0};//cam rotation
	dsSetViewpoint (posxyz, rotxyz);
}

void command (int key)
{
	switch (key)
	{
		case 'w':
			throttle = 1;
		break;

		case 's':
			throttle =-0.3;
		break;

		case ' ':
			throttle = 0;
		break;


		case 'd':
			steer += 0.2;
		break;
	
		case 'a':
			steer -= 0.2;
		break;
	}
}

void step (int pause)
{
	int loop;
	if (!pause)
	{
		//rear wheel motor
		for (loop=2; loop<4; loop++)
		{
			dJointAddHinge2Torques (car_wheel_joint[loop], 0, car_max_torque*throttle*direction);
		}

		//front wheel turning
		if (steer > car_max_turn) steer = car_max_turn;
		if (steer < -car_max_turn) steer = -car_max_turn;
		for (loop=0; loop<2; loop++)
		{
			dJointSetHinge2Param (car_wheel_joint[loop], dParamLoStop, steer*direction);
			dJointSetHinge2Param (car_wheel_joint[loop], dParamHiStop, steer*direction);
		}

		//simulation
		dSpaceCollide (track_space, 0, &collisioncallback); //get new contact joints
		dWorldQuickStep (track, 0.05); //TODO: add a timer system to make the simulation step realistic (fixed stepsize, and sleep)
		dJointGroupEmpty (contactgroup); //remove the contact joints
	}

	//draw the simulation
	dsSetColour (0, 1, 1); //car body
	dsSetTexture (DS_WOOD);
	dVector3 car_geoms = {car_width, car_lenght, car_height};
	dsDrawBox (dBodyGetPosition (car_body), dBodyGetRotation (car_body), car_geoms);
	
	dsSetColour (1, 1, 0); //car wheels
	for (loop=0; loop<4; loop++)
	{
		dsDrawCylinder (dBodyGetPosition (car_wheel_body[loop]), dBodyGetRotation (car_wheel_body[loop]),
				wheel_width, wheel_radius);
	}

	dsSetColour (0, 1, 0); //track objects
	dVector3 object_geoms;
	for (loop=1; loop<track_objects; loop++)
	{
		dGeomBoxGetLengths (track_object[loop], object_geoms);
		dsDrawBox (dGeomGetPosition (track_object[loop]), dGeomGetRotation (track_object[loop]), object_geoms);
	}
}

int main (int argc, char **argv)
{
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
	dMassSetBoxTotal (&mass, car_mass, car_width, car_lenght, car_height);
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
		dMassSetCylinderTotal (&mass, wheel_mass, 3, wheel_radius, wheel_width);
		dBodySetMass (car_wheel_body[loop], &mass);
		car_wheel_geom[loop] = dCreateCylinder (car_space, wheel_radius, wheel_width);
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
		dJointSetHinge2Param (car_wheel_joint[loop], dParamSuspensionERP, car_suspension_erp);
		dJointSetHinge2Param (car_wheel_joint[loop], dParamSuspensionCFM, car_suspension_cfm);
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

	//run race
	dsSimulationLoop (argc, argv, 640, 480, &drawfunc);

	//after race cleanup
	dJointGroupDestroy (car_joint_group);
	dSpaceDestroy (car_space);

	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (track_space);
	dWorldDestroy (track);
	dCloseODE();
	return 0;
}

