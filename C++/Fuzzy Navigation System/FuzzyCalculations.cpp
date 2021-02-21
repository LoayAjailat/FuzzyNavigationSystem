#include "FuzzyCalculations.h"

double FuzzyCalculations::Defuzzify(vector<double> fs, int* rules, vector <double> vels, int size)
{
	double fuzzyVel = 0, velocity = 0, sumOfDegrees = 0.0, result = 0.0;

	for (int i = 0; i < size; i++) {
		// Select speed of rule
		velocity = vels[rules[i]];
		// Calculate the sum of fuzzified values
		fuzzyVel = fuzzyVel + fs[i] * velocity;
		// Calculate the sum of firing strengths (degrees)
		sumOfDegrees = sumOfDegrees + fs[i];
	}

	// Defuzzify values
	result = fuzzyVel / sumOfDegrees;

	return result;
}

vector<double> FuzzyCalculations::CalcMemDegrees(double* dist, MembershipFunction* Sets[], int numSensors, int numSets)
{
	int numDegrees = numSensors * numSets;
	vector<double> memDegrees(numDegrees);
	int count = 0;
	for (int m = 0; m < numSets; m++) {
		for (int n = 0; n < numSensors; n++) {
			memDegrees[count] = Sets[m]->calcMembershipDegree(dist[n]);
			count++;
		}
	}
	return memDegrees;
}

vector<double> FuzzyCalculations::CalcFiringStrength(double* dist, vector<double> memDegrees, int numSensors, int numSets)
{
	int numDegrees = numSensors * numSets;
	int numResults = (int)pow(numSets, numSensors);
	vector<double> fs(numResults);

	int count = 0;
	for (int k = 0; k < numDegrees; k += 2) {
		for (int j = 1; j < numDegrees; j += 2) {
			fs[count] = memDegrees[k] * memDegrees[j];
			count++;
		}
	}
	return fs;
}

//Function to get the maximum
double FuzzyCalculations::maximum(int a, int b, int c)
{
	double max = (a > b) ? a : b;
	return ((max > c) ? max : c);
}

//Function to get the minimum
double FuzzyCalculations::minimum(int a, int b, int c)
{
	double min = (a > b) ? b : a;
	return ((min > c) ? c : min);
}

