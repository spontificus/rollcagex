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

//car conf
typedef struct {
	dReal mass;
	dReal max_torque;
	dReal suspension_erp;
	dReal suspension_cfm;

	dReal lenght;
	dReal width;
	dReal height;
} car_struct;

car_struct car[1]; //just one car

typedef struct {
	dReal stepsize;

	dReal wheel_radius;
	dReal wheel_width;
} internal_struct;

internal_struct internal;

//#define car_mass     40
//#define car_max_torque 500
//#define car_suspension_erp 0.9
//#define car_suspension_cfm 0.05
//and possibly a 3:rd file for this wheel data:
//#define wheel_radius 2
//#define wheel_width  2.5
#define wheel_mass   5
#define wheel_slip   0.05

//maybe not this data
#define wheel_front_position 3.5
#define wheel_rear_position -3.5
#define wheel_right_position 4.75
#define wheel_left_position -4.75

#define car_max_steer 0.6
#define car_turn_speed 0.1
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

struct {
	dReal throttle;
	dReal steer;
} control, control_real;

dReal direction;
