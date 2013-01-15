//********************************//
//   TECHNIK AUTONOMER SYSTEME    //
//********************************//

#include "playerlaser.h"
#include "playerposition2d.h"
#include "playerplanner.h"
#include "playermanager.h"
#include "playerlocalization.h"
#include "playerglobals.h"
#include "playergripper.h"
#include "playerblobfinder.h"
#include "robotmanipulation.h"
#include "MssArgs.h"
#include "time.h"

#define LASER_INDEX 				0
#define LASER_INTERPOLATOR_INDEX	1
#define SONAR_INDEX					0
#define AMCL_LOCALIZE_INDEX			0
#define POSITION2D_INDEX 			0
#define VFH_POSITION_INDEX			1
#define VFH_POSE_INDEX				2
#define AMCL_POSITION_INDEX			3
#define GRIPPER_INDEX				0
#define BLOBFINDER_INDEX			0
#define PTZ_INDEX					0


using namespace std;

void handle_plannererror(ePlannerError e) {
	switch (e) {
		case PLANNER_START_INSIDE_OBS:
			std::cout << "PlannerError: start inside obs\n";
			break;
		case PLANNER_START_OUTSIDE_MAP:
			std::cout << "PlannerError: start outside map\n";
			break;
		case PLANNER_GOAL_INSIDE_OBS:
			std::cout << "PlannerError: goal inside obs\n";
			break;
		case PLANNER_GOAL_OUTSIDE_MAP:
			std::cout << "PlannerError: goal outside map\n";
			break;
		case PLANNER_START_GOAL_IDENTIC:
			std::cout << "PlannerError: start and goal are in the same cell\n";
			break;
		case PLANNER_NO_PATH_FOUND:
			std::cout << "PlannerError: no path could be found\n";
			break;
		default:
			std::cout << "PlannerError: unknown error occured!\n";
			break;
	}
}

/*
 *
 * The goal points chosen in this demo, are selected by using the tas-test map!
 *
 */

int main(int argc, char *argv[]) {
	MssArgs * args = new MssArgs(argc, argv);

	// Create PlayerManager			
	PlayerManager *pm = new PlayerManager(10, args->playerHostname, args->playerPort, args->verbosity);

	// Create PositionDevice
	PlayerPosition2D *pp2d = new PlayerPosition2D(pm, POSITION2D_INDEX, VFH_POSE_INDEX, POSITION2D_INDEX,
			args->verbosity);
	PlayerLaser pl(pm, LASER_INDEX, args->verbosity);

	PlayerBlobfinder *pbf = new PlayerBlobfinder(pm, BLOBFINDER_INDEX, args->verbosity);

	PlayerGripper *pg = new PlayerGripper(pm, GRIPPER_INDEX, args->verbosity);
	PlayerPtz *ptz = new PlayerPtz(pm, PTZ_INDEX, args->verbosity);
	// Create MapDevice
	PlayerMap *pmap = new PlayerMap(pm, 0, args->verbosity);
	// Create PlannerDevice
	PlayerPlanner *pplan = new PlayerPlanner(pm, pp2d, pmap, POSITION2D_INDEX, true, args->verbosity, 0.3);

	RobotManipulation *rm = new RobotManipulation(pm, pp2d, pg, pbf, ptz, args->verbosity);

	//parameter
	BlobColorType color1 = GREEN;
	BlobColorType color2 = RED;
	playerc_blobfinder_blob_t blob;

	// Enable motors
	pp2d->powerUp();
	// Initialize Odometry to (0,0,0)
	pp2d->setOdometry(args->startX, args->startY, DTOR(args->startPhi));
	//cout << "plocpos\t" << pose[0] << "\t" << pose[1] << std::endl;
	cout << "pp2dpos\t" << pp2d->getXPos() << "\t" << pp2d->getYPos() << std::endl;

	//2-dimensional array of search points
	clock_t start, finish;
	double duration = 0;
	double startX = -1.0;
	double startY = -1.0;
	double searchPoint[5][2] = {{startX,startY},{1,-1},{1,1},{-1,1},{-1,0}};

	int pointNum = 0;

	// main loop

	while( pointNum < 5 )
	{
		duration = 0;
		start = clock();
		while(!pbf->blobFound(color1)&&!pbf->blobFound(color2)&&duration<5)
		{
			cout << "No Blob! Searching!" << endl;
			pp2d->setSpeed(0,0.5);
			pm->waitForData();
			finish = clock();
			duration = ((double)(finish - start))/10000;
			printf("%f seconds\n", duration);
		}
		pp2d->stop();

		if(pbf->blobFound(color1)||pbf->blobFound(color2))
		//if(false)
		{
			while(pbf->blobFound(color1)||pbf->blobFound(color2))
			{
				pbf->getBiggestBlob(color1, blob);
				cout << blob.x << "\t" << blob.y << "\t" <<(int) blob.area << endl;
				if ((int) blob.x < 35)
					pp2d->setSpeed(0, 1);
				else if ((int) blob.x > 45)
					pp2d->setSpeed(0, -0.13);
				else
				{
					cout << "Blob found! Moving towards!" << endl;
					pp2d->setSpeed(0.2,0);
				}
				if(pg->getOuterBreakBeam())
				{
					pp2d->setSpeed(0.2,0);
					sleep(1);
					break;
				}
			}
			pp2d->stop();
			pg->close();
			while(!pg->getClose())
			{
				printf("waiting til closed\n");
				sleep(1);
			}
			pg->moveUp();
			while (pg->getPos() !=1)
			{
				printf("lift moving up\n");
				sleep(1);
			}
			//printf("lift moving up\n");
			//try to grab
			//rm->grabPuck(color1);
			//while(!rm->grabFinished())
			//{
			//	pm->waitForData();
			//}
			//bring it back to start position
			pplan->goToPosition(startX, startY);

			while(!pplan->getReached())
			{
				pm->waitForData();
				cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;
			}

			cout<< "Attention!!" << endl;

			pp2d->stop();
			pp2d->goToPose(startX,startY,DTOR(180));
			while(!pp2d->getReached())
			{
				//cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;
				//cout << "not" << endl;
				if (pp2d->getPhi() < -3) 
				{	
		   			pp2d->setSpeed(0.0, -0.12);
					sleep(1);
					break;
				}	

				if (pp2d->getPhi() > 3) 
				{	
		   			pp2d->setSpeed(0.0, 0.12);
					sleep(1);
					break;
				}	
			}

			pp2d->stop();
			cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;
			while(1)
			{

			}

			printf("lift moving down\n");
			pg->moveDown();
			printf("open\n");
			pg->open();

			cout << "Backing off..." << endl;
			pp2d->setSpeed(-0.1,0);
			sleep(2);

			cout << "Job's done!"<< endl;
			pp2d->setSpeed(0, DTOR(-90));
			sleep(1);
			pp2d->setSpeed(0, 0);

			pplan->goToPosition(searchPoint[pointNum][0], searchPoint[pointNum][1]);

			while(!pplan->getReached())
			{
				pm->waitForData();
			}
		}

		//find no Blob

//		if(!pbf->blobFound(color1)&&!pbf->blobFound(color2))
		else
		{
			cout << "no blob! Let's go to next point!\n" << endl;
			//next position
			pointNum = pointNum + 1; 	
			pplan->goToPosition(searchPoint[pointNum][0], searchPoint[pointNum][1]);
			while(!pplan->getReached())
			{
				//find the position on the road
				if(pbf->blobFound(color1)||pbf->blobFound(color2))
				{
					pplan->abortGoTo();
					//bringBlobBack(startX,startY);
					while (true)
					{
						pbf->getBiggestBlob(color1, blob);

						cout << blob.x << "\t" <<(int) blob.area << endl;
						if ((int) blob.x < 35)
							pp2d->setSpeed(0, 0.13);
						else if ((int) blob.x > 45)
							pp2d->setSpeed(0, -0.13);
						else
						{
							cout << "Blob found! Moving towards!" << endl;
							pp2d->setSpeed(0.2,0);
						}
						if(pg->getOuterBreakBeam())
							break;
					}
					pp2d->stop();
					//try to grab
					rm->grabPuck(color1);
					while(!rm->grabFinished())
					{
						pm->waitForData();
					}
					//bring it back to start position
					pplan->goToPosition(startX, startY);

					while(!pplan->getReached())
					{
						pm->waitForData();
					}

					pp2d->stop();
					pg->moveDown();
					pg->open();

					cout << "Backing off..." << endl;
					pp2d->setSpeed(-0.1,0);
					sleep(2);

					cout << "Job's done!"<< endl;
				}
				pm->waitForData();

			}
		}	
	}

	// Goal reached -> stopping robot (safe solution, but robot should actually stop already on its own)
	pplan->goToPosition(startX, startY);

	while(!pplan->getReached())
	{
		pm->waitForData();
	}

	cout << "I am a Terminator!" << endl;
	pp2d->setSpeed(0, 0);

	// Disable motors
	pp2d->powerDown();

	// Cleaning up memory
	delete pplan;
	delete pmap;
	delete pp2d;
	delete pm;
	delete pbf;
	delete pg;
	delete ptz;
	delete rm;

	return 0;
}
