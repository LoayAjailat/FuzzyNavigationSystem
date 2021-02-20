#pragma once
#include <cstddef>
#include <string>
using namespace std;

class MembershipFunction
{
protected:
	double m_PointA, m_PointB, m_PointC, m_PointD;
	string m_Type;
	char* m_Name;

public:
	// Constructor
	MembershipFunction() {};

	// Destructor
	~MembershipFunction() { delete[] m_Name; m_Name = NULL; }

	// Functions
	void setShapeParameters(double a, double b, double c, double d);
	void defineShape(string type);
	void setName(const char* name);
	bool isWithinRange(double t);
	string getType(void)const;
	string getName() const;
	double calcMembershipDegree(double dist);
	double calcMidPoint();
};
