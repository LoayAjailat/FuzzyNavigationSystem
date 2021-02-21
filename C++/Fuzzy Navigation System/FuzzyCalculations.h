#pragma once
#include "MembershipFunction.h"
#include <vector>

using namespace std;
class FuzzyCalculations
{
public:
	double Defuzzify(vector<double> fs, int* rules, vector <double> vels, int size);
	vector <double> CalcMemDegrees(double* dist, MembershipFunction* Sets[], int numSensors, int numSets);
	vector<double> CalcFiringStrength(double* dist, vector <double> memDegrees, int numSensors, int numSets);
	double maximum(int a, int b, int c);
	double minimum(int a, int b, int c);
};

