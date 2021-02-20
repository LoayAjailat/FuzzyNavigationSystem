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
	double result = 0, v = 0;

	double arr[3] = { midSlow, midMedium, midFast };

	for (int i = 0; i < size; i++) {
		v = arr[rules[i]];
		result = result + fs[i] * v;
	}
	return result;
}

vector<double> CalcFiringStrength(double* dist, MembershipFunction* Sets[], int numSensors, int numSets)
{
	int numDegrees = numSensors * numSets;
	int numResults = (int)pow(numSets, numSensors);

	vector<double> fs(numResults); // why vector double? Fix
	vector<double> memDegrees(numDegrees);

	int count = 0;
	for (int m = 0; m < numSets; m++) {
		for (int n = 0; n < numSensors; n++) {
			memDegrees[count] = Sets[m]->calcMembershipDegree(dist[n]);
			count++;
		}
	}
	//// TODO: HAVE CHECKS FOR SIZES
	count = 0;
	for (int k = 0; k < numDegrees; k+=2) {
		for (int j = 1; j < numDegrees; j+=2) {
			fs[count] = memDegrees[k] * memDegrees[j];
			count++;
		}
	}

	return fs;
}

//double CalcFiringStrength{};

int main()
{
	SetParameters();
	GetMidPoints();
	
	MembershipFunction* FuzzySet[9] = { mf_Close, mf_Ideal, mf_Far, 
										mf_Slow, mf_Medium, mf_Fast,
										mf_OA, mf_RE, mf_GS };

	double dists[2] = { 105,570 };
	vector<double> fs = CalcFiringStrength(dists, FuzzySet, 2, 3);
	cout << fs[1] << endl;

	int size1 = sizeof(RE_L) / sizeof(RE_L[0]);
	int size2 = fs.size();

	if (size1 == size2)
	{
		double r = Defuzzify(fs, RE_L, size1);
		cout << "Def " << r << endl;
	}
}