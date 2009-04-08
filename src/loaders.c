#define maxchars 40 //(hopefully) enought space for comments ;)

int load_car_data (const char *filename, car_struct *carp)
{
	printf ("\n> \"load_car_data\" will load the car settings from \"%s\"\n", filename);

	char string[maxchars];
	FILE  *lstp = fopen (filename, "r");

	if (!lstp)
	{
		printf("file doesn't exist!\n");
		return 1;
	}

	float value;
	while (fgets (string, maxchars, lstp) != NULL)
	{
		printf ("> Reading String: %s> ", string);
		if (string[0] == '\n' || string[0] == '#') printf ("just a comment or blank line");

		else if (sscanf (string, "mass %f \n", &carp -> mass) == 1) printf ("mass got value %f", carp -> mass);
		else if (sscanf (string, "max_torque %f \n", &carp -> max_torque) == 1) printf ("max_torque got value %f", carp -> max_torque);
		else if (sscanf (string, "suspension_erp %f \n", &carp -> suspension_erp) == 1) printf ("suspension_erp got value %f", carp -> suspension_erp);
		else if (sscanf (string, "suspension_cfm %f \n", &carp -> suspension_cfm) == 1) printf ("suspension_cfm got value %f", carp -> suspension_cfm);

		else if (sscanf (string, "lenght %f \n", &carp -> lenght) == 1) printf ("lenght got value %f", carp -> lenght);
		else if (sscanf (string, "width %f \n", &carp -> width) == 1) printf ("width got value %f", carp -> width);
		else if (sscanf (string, "height %f \n", &carp -> height) == 1) printf ("height got value %f", carp -> height);

		else
		{
			printf ("ERROR: line is not a comment, nor a value definition!\n");
			fclose (lstp);
			return 1;
		}

		putchar ('\n');
	}
	fclose (lstp);

	printf ("> done\n\n\n");

	return 0;
}

int load_internal_data (const char *filename, internal_struct *internalp)
{
	printf ("\n> \"load_internal_data\" will load settings from \"%s\"\n", filename);

	char string[maxchars];
	FILE  *lstp = fopen (filename, "r");

	if (!lstp)
	{
		printf("file doesn't exist!\n");
		return 1;
	}

	float value;
	while (fgets (string, maxchars, lstp) != NULL)
	{
		printf ("> Reading String: %s> ", string);
		if (string[0] == '\n' || string[0] == '#') printf ("just a comment or blank line");

		else if (sscanf (string, "stepsize %f \n", &internalp -> stepsize) == 1) printf ("stepsize got value %f", internalp -> stepsize);

		else if (sscanf (string, "wheel_radius %f \n", &internalp -> wheel_radius) == 1) printf ("wheel_radius got value %f", internalp -> wheel_radius);
		else if (sscanf (string, "wheel_width %f \n", &internalp -> wheel_width) == 1) printf ("wheel_width got value %f", internalp -> wheel_width);

		else
		{
			printf ("ERROR: line is not a comment, nor a value definition!\n");
			fclose (lstp);
			return 1;
		}

		putchar ('\n');
	}
	fclose (lstp);

	printf ("> done\n\n\n");

	return 0;
}
