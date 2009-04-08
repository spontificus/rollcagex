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

void simulate_race(dReal size)
{
	int loop;

	for (loop=0; loop<4; loop++)
	{
		dBodySetFiniteRotationAxis (car_wheel_body[loop], 0, 0, 0);
	}

	//rear wheel motor
	control_real.throttle = control.throttle;

	for (loop=2; loop<4; loop++)
	{
		dJointAddHinge2Torques (car_wheel_joint[loop], 0, car[0].max_torque*control_real.throttle*direction);
	}

	//front wheel turning
	control_real.steer = control.steer;
	
	for (loop=0; loop<2; loop++)
	{
		dJointSetHinge2Param (car_wheel_joint[loop], dParamLoStop, control_real.steer*direction);
		dJointSetHinge2Param (car_wheel_joint[loop], dParamHiStop, control_real.steer*direction);
	}

	//simulation
	dSpaceCollide (track_space, 0, &collisioncallback); //get new contact joints
	dWorldQuickStep (track, size); //do simulation
	dJointGroupEmpty (contactgroup); //remove the contact joints
}
