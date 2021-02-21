#include "MembershipFunction.h"
#include "FuzzyRules.h"
#include "FuzzyCalculations.h"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

/// <summary>
/// Global object and variable definitions
/// </summary>

// Distance membership functions
///////WHY ARE WE USING * TO DEFINE
MembershipFunction* mf_Close  = new MembershipFunction();
MembershipFunction* mf_Ideal  = new MembershipFunction();
MembershipFunction* mf_Far	  = new MembershipFunction();
// Speed membership functions
MembershipFunction* mf_Slow	  = new MembershipFunction();
MembershipFunction* mf_Medium = new MembershipFunction();
MembershipFunction* mf_Fast   = new MembershipFunction();
// Objectives membership functions
MembershipFunction* mf_OA	  = new MembershipFunction(); // Obstacle Avoidance
MembershipFunction* mf_RE	  = new MembershipFunction(); // Right-edge seeking
MembershipFunction* mf_GS	  = new MembershipFunction(); // Goal-seeking

double midSlow, midMedium, midFast;

// Set & define the shapes of the membership functions
void SetParameters()
{
	// Distance membership functions
	mf_Close->setShapeParameters(0, 100, 275, 450);
	mf_Close->defineShape("Trapezoid");
	mf_Close->setName("Close");

	mf_Ideal->setShapeParameters(400, 575, 575, 750);
	mf_Ideal->defineShape("Triangle");
	mf_Ideal->setName("Ideal");

	mf_Far->setShapeParameters(725, 1100, 1475, 5000);
	mf_Far->defineShape("Trapezoid");
	mf_Far->setName("Far");

	// Speed membership functions
	mf_Slow->setShapeParameters(0, 40, 60, 125);
	mf_Slow->defineShape("Trapezoid");
	mf_Slow->setName("Slow");

	mf_Medium->setShapeParameters(80, 140, 160, 200);
	mf_Medium->defineShape("Trapezoid");
	mf_Medium->setName("Medium");

	mf_Fast->setShapeParameters(160, 200, 240, 280);
	mf_Fast->defineShape("Trapezoid");
	mf_Fast->setName("Fast");

	// Objective membership functions
	mf_OA->setShapeParameters(0, 0, 750, 2000);
	mf_OA->defineShape("Trapezoid");
	mf_OA->setName("OA");

	mf_RE->setShapeParameters(0, 0, 800, 2500);
	mf_RE->defineShape("Trapezoid");
	mf_RE->setName("RE");

	mf_GS->setShapeParameters(1800, 3000, 5000, 5000);
	mf_GS->defineShape("Trapezoid");
	mf_GS->setName("GS");
}

// Gets the midpoint between points B & C of each shape
void GetMidPoints()
{
	// Gets the mid-points for the speed shapes
	midSlow = mf_Slow->calcMidPoint();
	midMedium = mf_Ideal->calcMidPoint();
	midFast = mf_Fast->calcMidPoint();
}

//Function to get the maximum
double maximum(int a, int b, int c)
{
	double max = (a > b) ? a : b;
	return ((max > c) ? max : c);
}

//Function to get the minimum
double minimum(int a, int b, int c)
{
	double min = (a > b) ? b : a;
	return ((min > c) ? c : min);
}

// Truncates the value if its equal to or exceeds the max
void truncate(double &val, double max) 
{
	if (val >= max)
		val -= 1;
}

int main()
{
	SetParameters();
	GetMidPoints();
	
	MembershipFunction* FuzzySet[9] = { mf_Close, mf_Ideal, mf_Far, 
										mf_Slow, mf_Medium, mf_Fast,
										mf_OA, mf_RE, mf_GS };

	MembershipFunction* DistanceSets[3]  = { FuzzySet[0], FuzzySet[1], FuzzySet[2] };
	MembershipFunction* SpeedSets[3]	 = { FuzzySet[3], FuzzySet[4], FuzzySet[5] };
	MembershipFunction* ObjectiveSets[3] = { FuzzySet[6], FuzzySet[7], FuzzySet[8] };

	//DUMMY DATA
	// get sonar readings
	double sonarRange[8] = { 0, 0, 400, 400, 500, 600, 300, 500 };

	double sonar7 = sonarRange[7];
	double sonar6 = sonarRange[6];
	double sonar5 = sonarRange[5];
	double sonar4 = sonarRange[4];
	double sonar3 = sonarRange[3];
	double sonar2 = sonarRange[2];

	truncate(sonar2, 5000);
	truncate(sonar3, 5000);
	truncate(sonar4, 5000);
	truncate(sonar5, 5000);
	truncate(sonar6, 5000);
	truncate(sonar7, 5000);

	// Check with isWithinRange function. Maybe truncate within it?

	double sonarFront = min(sonar3, sonar4);

	double RE_sensors[2] = { sonar6, sonar7 };
	double OA_sensors[3] = { sonar2, sonarFront, sonar5 };

	double rightVel_RE, leftVel_RE, rightVel_OA, leftVel_OA, leftVelFinal, rightVelFinal;
	vector <double> a_midVel{ midSlow, midMedium, midFast };

	FuzzyCalculations FC; // Whats the difference with --> FuzzyCalculations* FC = new FuzzyCalculations();
	 
	// Right Edge following
	int numSets = sizeof(DistanceSets) / sizeof(DistanceSets[0]);
	int numSensors = sizeof(RE_sensors) / sizeof(RE_sensors[0]);
	vector <double> md_RE = FC.CalcMemDegrees(RE_sensors, DistanceSets, numSensors, numSets);
	vector <double> fs_RE = FC.CalcFiringStrength(RE_sensors, md_RE, numSensors, numSets);
	
	int a_Size = sizeof(RE_R) / sizeof(RE_R[0]);
	int fs_Size = fs_RE.size();
	if (a_Size == fs_Size) {
		//Check if the size of both arrays match
		rightVel_RE = FC.Defuzzify(fs_RE, RE_R, a_midVel, a_Size);
		leftVel_RE = FC.Defuzzify(fs_RE, RE_L, a_midVel, a_Size);
	}

	// Obstacle avoidance
	numSensors = sizeof(OA_sensors) / sizeof(OA_sensors[0]);
	vector <double> md_OA = FC.CalcMemDegrees(OA_sensors, DistanceSets, numSensors, numSets);
	vector <double> fs_OA = FC.CalcFiringStrength(OA_sensors, md_OA, numSensors, numSets);

	a_Size = sizeof(OA_R) / sizeof(OA_R[0]);
	fs_Size = fs_OA.size();
	if (a_Size == fs_Size) {
		//Check if the size of both arrays match
		rightVel_OA = FC.Defuzzify(fs_OA, OA_R, a_midVel, a_Size);
		leftVel_OA = FC.Defuzzify(fs_OA, OA_L, a_midVel, a_Size);
	}

	cout << "RE Velocity: " << rightVel_RE << " & " << leftVel_RE << endl;
	cout << "OA Velocity: " << rightVel_OA << " & " << leftVel_OA << endl;
	
	double minSensor_OA, memDegree_OA;
	double minSensor_RE, memDegree_RE, memDegree_GS;

	minSensor_OA = minimum(sonar5, sonarFront, sonar2);
	memDegree_OA = ObjectiveSets[0]->calcMembershipDegree(minSensor_OA);

	minSensor_RE = min(sonar6, sonar7);
	memDegree_RE = ObjectiveSets[1]->calcMembershipDegree(minSensor_RE);
	memDegree_GS = ObjectiveSets[2]->calcMembershipDegree(minSensor_RE);
	memDegree_RE = max(memDegree_RE, memDegree_GS) * 0.8;

	leftVelFinal = (memDegree_OA * leftVel_OA + memDegree_RE * leftVel_RE) / (memDegree_OA + memDegree_RE);
	rightVelFinal = (memDegree_OA * rightVel_OA + memDegree_RE * rightVel_RE) / (memDegree_OA + memDegree_RE);

	cout << "OA: " << memDegree_OA << " " << "RE: " << memDegree_RE << endl;
	cout << "Right: " << rightVelFinal << " " << "Left: " << leftVelFinal << endl;

	//TODO: Make some doubles as float. No need for all that precision
}