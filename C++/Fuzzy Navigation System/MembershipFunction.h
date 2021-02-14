#pragma once
#include <cstddef>
class MembershipFunction
{
protected:
	double m_PointA, m_PointB, m_PointC, m_PointD;
	char  m_Type;
	char* m_Name;

public:
	// Constructor
	MembershipFunction() {};

	// Destructor
	~MembershipFunction() { delete[] m_Name; m_Name = NULL; }

	// Functions
	void setShapeParameters(double a, double b, double c, double d);
	void setType(char type);
	void setName(const char* name);
	bool isWithinRange(double t);
	char getType(void)const;
	void getName() const;
	double calcMembershipDegree(double dist);
	double calcMidPoint();
};
