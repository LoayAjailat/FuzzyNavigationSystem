#include "MembershipFunction.h"
#include "FuzzyRules.h"
#include <iostream>
#include <vector>
#include <algorithm>    // std::transform

using namespace std;

/// <summary>
/// Global object and variable definitions
/// </summary>

// Distance membership functions
///////WHY ARE WE USING * TO DEFINE
MembershipFunction* mf_Close = new MembershipFunction();
MembershipFunction* mf_Ideal = new MembershipFunction();
MembershipFunction* mf_Far = new MembershipFunction();
// Speed membership functions
MembershipFunction* mf_Slow = new MembershipFunction();
MembershipFunction* mf_Medium = new MembershipFunction();
MembershipFunction* mf_Fast = new MembershipFunction();
// Objectives membership functions
MembershipFunction* mf_OA = new MembershipFunction(); // Obstacle Avoidance
MembershipFunction* mf_RE = new MembershipFunction(); // Right-edge seeking
MembershipFunction* mf_GS = new MembershipFunction(); // Goal-seeking

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

void GetMidPoints()
{
	// Gets the mid-points for the speed shapes
	midSlow = mf_Slow->calcMidPoint();
	midMedium = mf_Ideal->calcMidPoint();
	midFast = mf_Fast->calcMidPoint();
}

double Defuzzify(vector<double> &fs, int* rules, int size) 
{
	double fuzzyVel = 0, velocity = 0, sumOfDegrees = 0.0, result = 0.0;

	double arr[3] = { midSlow, midMedium, midFast };

	for (int i = 0; i < size; i++) {
		// Select speed of rule
		velocity = arr[rules[i]];
		// Calculate the sum of fuzzified values
		fuzzyVel = fuzzyVel + fs[i] * velocity;
		// Calculate the sum of membership degrees
		sumOfDegrees = sumOfDegrees + fs[i];
	}

	// Defuzzify values
	result = fuzzyVel / sumOfDegrees;
	
	return result;
}

vector<double> CalcFiringStrength(double* dist, MembershipFunction* Sets[], int numSensors, int numSets)
{
	//// TODO: HAVE CHECKS FOR SIZES HERE OR OUTSIDE

	int numDegrees = numSensors * numSets;
	int numResults = (int)pow(numSets, numSensors);

	vector<double> fs(numResults); // why vector double? Fix
	vector<double> memDegrees(numDegrees);

	// Calculates the membership degrees of the sensors
	int count = 0;
	for (int m = 0; m < numSets; m++) {
		for (int n = 0; n < numSensors; n++) {
			memDegrees[count] = Sets[m]->calcMembershipDegree(dist[n]);
			count++;
		}
	}
	// Calculate the firing strength of each rule
	count = 0;
	for (int k = 0; k < numDegrees; k+=2) {
		for (int j = 1; j < numDegrees; j+=2) {
			fs[count] = memDegrees[k] * memDegrees[j];
			count++;
		}
	}

	return fs;
}

//double GetSpeeds(vector<double>& fs, int* rulesR, int* rulesL, int size) {
//	double rightVel = 0.0, leftVel = 0.0;
//
//	rightVel = Defuzzify(fs, rulesR, size);
//	cout << "Right wheel velocity: " << rightVel << endl;
//
//	leftVel = Defuzzify(fs, rulesL, size);
//	cout << "Left wheel velocity: " << leftVel << endl;
//
//	return rightVel, leftVel;
//}

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
	double sonarFront = min(sonar3, sonar4);

	double RE_sensors[2] = { sonar6, sonar7 };
	double OA_sensors[3] = { sonar2, sonarFront, sonar5 };

	double rightVel_RE = 0.0, leftVel_RE = 0.0, rightVel_OA = 0.0, leftVel_OA = 0.0;

	// Right Edge following
	int numSets = sizeof(DistanceSets) / sizeof(DistanceSets[0]);
	int numSensors = sizeof(RE_sensors) / sizeof(RE_sensors[0]);
	vector<double> fs_RE = CalcFiringStrength(RE_sensors, DistanceSets, numSensors, numSets);
	
	int a_Size = sizeof(RE_R) / sizeof(RE_R[0]);
	int fs_Size = fs_RE.size();
	if (a_Size == fs_Size) {
		//Check if the size of both arrays match
		rightVel_RE = Defuzzify(fs_RE, RE_R, a_Size);
		leftVel_RE = Defuzzify(fs_RE, RE_L, a_Size);
	}

	// Obstacle avoidance
	numSensors = sizeof(OA_sensors) / sizeof(OA_sensors[0]);
	vector<double> fs_OA = CalcFiringStrength(OA_sensors, DistanceSets, numSensors, numSets);

	a_Size = sizeof(OA_R) / sizeof(OA_R[0]);
	fs_Size = fs_OA.size();
	if (a_Size == fs_Size) {
		//Check if the size of both arrays match
		rightVel_OA = Defuzzify(fs_OA, OA_R, a_Size);
		leftVel_OA = Defuzzify(fs_OA, OA_L, a_Size);
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

	double leftVelFinal = (memDegree_OA * leftVel_OA + memDegree_RE * leftVel_RE) / (memDegree_OA + memDegree_RE);
	double rightVelFinal = (memDegree_OA * rightVel_OA + memDegree_RE * rightVel_RE) / (memDegree_OA + memDegree_RE);

	cout << "OA: " << memDegree_OA << " " << "RE: " << memDegree_RE << endl;
	cout << "Left: " << leftVelFinal << " " << "Right: " << rightVelFinal << endl;
}