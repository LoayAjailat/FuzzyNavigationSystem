#include "MembershipFunction.h"
#include <iostream>
using namespace std;

/*
*  Dependencies
*/

// Distance membership functions
MembershipFunction* Close = new MembershipFunction();
MembershipFunction* Ideal = new MembershipFunction();
MembershipFunction* Far = new MembershipFunction();
// Speed membership functions
MembershipFunction* Slow = new MembershipFunction();
MembershipFunction* Medium = new MembershipFunction();
MembershipFunction* Fast = new MembershipFunction();
// Objectives membership functions
MembershipFunction* OA = new MembershipFunction(); // Obstacle Avoidance
MembershipFunction* RE = new MembershipFunction(); // Right-edge seeking
MembershipFunction* GS = new MembershipFunction(); // Goal-seeking

double midSlow, midMedium, midFast;

// Set the shapes of the membership functions
void SetParameters()
{
	Close->setShapeParameters(0, 100, 275, 450);
	Ideal->setShapeParameters(400, 575, 575, 750);
	Far->setShapeParameters(725, 1100, 1475, 5000);

	Slow->setShapeParameters(0, 40, 60, 125);
	Medium->setShapeParameters(80, 140, 160, 200);
	Fast->setShapeParameters(160, 200, 240, 280);

	OA->setShapeParameters(0, 0, 750, 2000);
	RE->setShapeParameters(0, 0, 800, 2500);
	GS->setShapeParameters(1800, 3000, 5000, 5000);
}

void GetMidPoints()
{
	// Gets the mid-points for the speed shapes
	midSlow = Slow->calcMidPoint();
	midMedium = Ideal->calcMidPoint();
	midFast = Fast->calcMidPoint();
}

int main()
{
	SetParameters();
}