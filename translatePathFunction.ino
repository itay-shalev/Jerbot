void translatePath(char directions[])
{
	int i = 0;
	int driveDir = FORWARD;
	for (i = 0; directions[i + 1] != NULL; i++)
	{
		switch (directions[i])
		{
		case 'U':
			driveDir = FORWARD;
			break;
		case 'R':
			driveDir = RIGHT;
			break;
		case 'L':
			driveDir = LEFT;
		case 'D':
			driveDir = BACKWARD;
			break;
		default:
			driveDir = FORWARD;
		}
		switch (directions[i + 1])
		{
		case 'U':
			wallDrive(dSpeed, driveDir, BOTH); //TODO: wallDrive is supposed to check for intersections and not for one wall/face.]
			break;
		case 'R':
			wallDrive(dSpeed, driveDir, RIGHT);
			break;
		case 'L':
			wallDrive(dSpeed, driveDir, LEFT);
			break;
		case 'D':
			wallDrive(dSpeed, driveDir, BOTH);
		}
	}
}