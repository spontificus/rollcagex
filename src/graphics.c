void draw_race(void)
{
	int loop;
	//draw the simulation
	dsSetColour (0, 1, 1); //car body
	dsSetTexture (DS_WOOD);
	dVector3 car_geoms = {car_width, car_lenght, car_height};
	dsDrawBox (dBodyGetPosition (car_body), dBodyGetRotation (car_body), car_geoms);
	
	dsSetColour (1, 1, 0); //car wheels
	for (loop=0; loop<4; loop++)
	{
		dsDrawCylinder (dBodyGetPosition (car_wheel_body[loop]), dBodyGetRotation (car_wheel_body[loop]),
				internal.wheel_width, internal.wheel_radius);
	}

	dsSetColour (0, 1, 0); //track objects
	dVector3 object_geoms;
	for (loop=1; loop<track_objects; loop++)
	{
		dGeomBoxGetLengths (track_object[loop], object_geoms);
		dsDrawBox (dGeomGetPosition (track_object[loop]), dGeomGetRotation (track_object[loop]), object_geoms);
	}
}
