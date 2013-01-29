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
	PlayerPosition2D *pp2d = new PlayerPosition2D(pm, POSITION2D_INDEX, VFH_POSE_INDEX, AMCL_LOCALIZE_INDEX, args->verbosity);
	//PlayerPosition2D *pp2d = new PlayerPosition2D(pm, POSITION2D_INDEX, VFH_POSE_INDEX, POSITION2D_INDEX, args->verbosity);
	//PlayerPosition2D *pp2d = new PlayerPosition2D(pm, POSITION2D_INDEX, VFH_POSE_INDEX, VFH_POSITION_INDEX, args->verbosity);

	PlayerLaser pl(pm, LASER_INDEX, args->verbosity);

	PlayerBlobfinder *pbf = new PlayerBlobfinder(pm, BLOBFINDER_INDEX, args->verbosity);

	PlayerGripper *pg = new PlayerGripper(pm, GRIPPER_INDEX, args->verbosity);
	PlayerPtz *ptz = new PlayerPtz(pm, PTZ_INDEX, args->verbosity);
	// Create MapDevice
	PlayerMap *pmap = new PlayerMap(pm, 0, args->verbosity);
	// Create PlannerDevice
	//PlayerPlanner *pplan = new PlayerPlanner(pm, pp2d, pmap, POSITION2D_INDEX, true, args->verbosity, 0.3);

	//PlayerLocalization *ploc = new PlayerLocalization(pm, pp2d, 100, AMCL_LOCALIZE_INDEX, args->verbosity);

	PlayerPlanner *pplan = new PlayerPlanner(pm, pp2d, pmap, AMCL_LOCALIZE_INDEX, true, args->verbosity, 0.3);

	RobotManipulation *rm = new RobotManipulation(pm, pp2d, pg, pbf, ptz, args->verbosity);

	//parameter
	BlobColorType color1 = GREEN;
	BlobColorType color2 = RED;
	BlobColorType color = GREEN;
	playerc_blobfinder_blob_t blob;
	playerc_blobfinder_blob_t blob1;
	playerc_blobfinder_blob_t blob2;

	// Enable motors
	pp2d->powerUp();
	// Initialize Odometry to (0,0,0)
	pp2d->setOdometry(args->startX, args->startY, DTOR(args->startPhi));
	pg->moveDown();
	pg->open();
	//double pose[100][3] = {};
	//cout << ploc->getHypothCount() << endl;
	//ploc->localize(pp2d, pose);
	//ploc->getHypoth(1, pose);

	//cout << "plocpos\t" << pose[0] << "\t" << pose[1] << "\t" << pose[2] << std::endl;
	cout << "pp2dpos\t" << pp2d->getXPos() << "\t" << pp2d->getYPos() << std::endl;

	//2-dimensional array of search points
	clock_t start, finish;
	double duration = 0;
	double startX = args->startX;
	double startY = args->startY;
	//double searchPoint[4][2] = {{startX,startY},{1,-1},{1,1},{-1,1}};
	double searchPoint[3][2] = {{startX,startY},{-1.3,1.3},{-1.5,-1.5}};
	double goalX = 0;
	double goalY = 0;
	double phi = 0;
	bool flag = 0;

	int pointNum = 0;
	//int  windowWidth = 0;

	// main loop

	while( pointNum < 3 )
	{
		duration = 0;
		start = clock();
		flag = 0;

		//if the roboter on the start positon, just turn 1/4 circle
		if(abs(pp2d->getXPos()-startX)<0.2&&abs(pp2d->getYPos()-startY)<0.2)
		{
			pp2d->goToPose(startX,startY,DTOR(-90));
			while(!pp2d->getReached())
			{
				pm->waitForData();
			}
			//while(!pbf->blobFound(color1)&&!pbf->blobFound(color2)&&duration<1)
			cout << "No Blob! Searching!" << endl;
			pp2d->goToPose(startX,startY,DTOR(-180));
			while(!pp2d->getReached())
			{
				pm->waitForData();
				if((pbf->blobFound(color1)||pbf->blobFound(color2)))
				{
					if(pbf->blobFound(color1)&&abs((int)blob.area>80))
					{
						pbf->getBiggestBlob(color1, blob);
						flag = 1;
						break;
					}
					if(pbf->blobFound(color2)&&abs((int)blob.area>80))
					{
						pbf->getBiggestBlob(color2, blob);
						if((blob.right-blob.left)<(blob.bottom-blob.top))
						{
						flag = 1;
						break;
						}
					}
				}
			}
			pp2d->stop();
		}

		//if the roboter on the start positon, turn 1 circle
		else
		{
			while(duration<3)
			{
				cout << "No Blob! Searching!" << endl;
				pp2d->setSpeed(0,0.3);
				pm->waitForData();
				finish = clock();
				duration = ((double)(finish - start))/10000;
				printf("%f seconds\n", duration);

				if(pbf->blobFound(color1)||pbf->blobFound(color2))
				{
					if(pbf->blobFound(color1))
					{
						pbf->getBiggestBlob(color1, blob);
						if(blob.range<sqrt((startX-pp2d->getXPos())*(startX-pp2d->getXPos())+(startY-pp2d->getYPos())*(startY-pp2d->getYPos()))&&abs((int)blob.area)>80)
						{
							flag = 1;
							break;
						}

					}

					if(pbf->blobFound(color2))
					{
						pbf->getBiggestBlob(color2, blob);
						if(blob.range<sqrt((startX-pp2d->getXPos())*(startX-pp2d->getXPos())+(startY-pp2d->getYPos())*(startY-pp2d->getYPos()))&&abs((int)blob.area)>80)
						{
							if((blob.right-blob.left)<(blob.bottom-blob.top))
							{
								flag = 1;
								break;
							}
						}

					}
				}
			}
			pp2d->stop();
		}

		if((pbf->blobFound(color1)||pbf->blobFound(color2))&&flag)
			//if(false)
		{
			if(pbf->blobFound(color1)&&!pbf->blobFound(color2))
				color = color1;
			if(pbf->blobFound(color2)&&!pbf->blobFound(color1))
				color = color2;
			if(pbf->blobFound(color1)&&pbf->blobFound(color2)) 
			{
				pbf->getBiggestBlob(color1, blob1);
				pbf->getBiggestBlob(color2, blob2);
				if(blob1.range<blob2.range)
					color = color1;
				else
					color = color2;

			}
			//int blobwidth = (int) (blob.left - blob.right);
			//windowWidth = pbf->getWindowWidth();
			//cout << windowWidth << blob.x << "\t" << blob.y << "\t" << blob.area << "\t" << blob.left << "\t" << blob.right << "\t" << abs(blobwidth) << "\t" << blob.range << endl;
			while((int) blob.x != 40)
				//while((int) blob.x != 320)
			{
				pbf->getBiggestBlob(color, blob);
				if ((int) blob.x < 35)
					//if ((int) blob.x < 300)
					pp2d->setSpeed(0.0, 0.13);
				//else if ((int) blob.x > 350)
				else if ((int) blob.x > 45)
					pp2d->setSpeed(0.0, -0.12);
				cout << blob.x << "\t" << blob.y << "\t" << blob.range << endl;
				//cout << blob.left << endl;
				//cout << blob.right << endl;
				//cout << blob.right-blob.left  << endl;
				//cout << blob.bottom - blob.top << endl;
				cout << abs((int)blob.area) << endl;
			}
			goalX = pp2d->getXPos() + cos(pp2d->getPhi()-0.02506)*(blob.range+0.063);
			//goalX = pp2d->getXPos() + cos(pp2d->getPhi()-0.02506)*(blob.range-0.2);
			goalY = pp2d->getYPos() + sin(pp2d->getPhi()-0.02506)*(blob.range+0.063);
			//goalY = pp2d->getYPos() + sin(pp2d->getPhi()-0.02506)*(blob.range-0.2);

			phi = pp2d->getPhi()-0.02506;
			cout << "Blob found! Moving towards!" << endl;
			cout << "top\t" << blob.top<< endl;
			cout << "bottom\t" << blob.bottom<< endl;
			cout << "range\t" << blob.range << endl;
			cout << "phi\t" << DTOR(phi) << endl;
			cout << "cos\t" << cos(phi) << endl;
			cout << "sin\t" << sin(phi) << endl;
			cout << "x\t" << goalX << endl;
			cout << "y\t" << goalY << endl;
			//	cout << ploc->getHypothCount() << endl;

			try{
				pplan->goToPosition(goalX, goalY);
			} catch (ePlannerError e) {
				handle_plannererror(e);
			}

			while(!pplan->getReached())
			{

				cout << " " << endl;
				cout << pl[180] << endl;
				cout << pl[270] << endl;
				cout << pl[90] << endl;
				cout << pl[132] << endl;
				cout << pl[228] << endl;
				cout << pl[296] << endl;
				cout << pl[64] << endl;

				while(pl[180]<0.7||pl[270]<0.3||pl[90]<0.3||pl[132]<0.7||pl[228]<0.7||pl[296]<0.3||pl[64]<0.3)
				{
					pp2d->stop();
				}
				if(pg->getInnerBreakBeam())
				{
					pplan->abortGoTo();
				}
			}


			if(pg->getOuterBreakBeam()||pg->getInnerBreakBeam())
			{
				pp2d->stop();
				pg->close();
				while(!pg->getClose())
				{
					printf("waiting til closed\n");
					sleep(1);
				}
				pg->moveUp();
				while (!pg->getPos())
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
				//			pplan->goToPosition(startX, startY);
				cout << "plan to home" << endl;

				while(!pplan->getReached())
				{
					pm->waitForData();
					//cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;
				}

				cout<< "Attention!!" << endl;
				cout << "start to put down" << endl;

				pp2d->stop();
				pp2d->goToPose(startX,startY,DTOR(45));
				while(!pp2d->getReached())
				{
					pm->waitForData();
					//cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;
					//cout << "not" << endl;
				}

				pp2d->stop();
				cout<< pp2d->getPhi() << "\t" << pp2d->getXPos() << "\t" <<pp2d->getYPos() << endl;

				pp2d->setSpeed(0.1,0);
				sleep(3);

				printf("lift moving down\n");
				pg->moveDown();
				printf("open\n");
				pg->open();

				cout << "Backing off..." << endl;
				pp2d->setSpeed(-0.15,0);
				sleep(3);
				pp2d->stop();

				cout << "Job's done!"<< endl;
				pplan->goToPosition(searchPoint[pointNum][0], searchPoint[pointNum][1]);

				while(!pplan->getReached())
				{
					cout << "go to position" << endl;
					pm->waitForData();
				}
			}
		}

		//find no Blob

		//		if(!pbf->blobFound(color1)&&!pbf->blobFound(color2))
		else
		{
			//next position
			pointNum = pointNum + 1; 	
			if(pointNum > 2)
				break;
			cout << "go to " << pointNum+1 <<". position" << endl;
			pplan->goToPosition(searchPoint[pointNum][0], searchPoint[pointNum][1]);
			//pp2d->goToPosition(searchPoint[pointNum][0], searchPoint[pointNum][1]);
			while(!pplan->getReached())
				//while(!pp2d->getReached())
			{
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
