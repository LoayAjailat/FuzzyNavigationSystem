#include "MembershipFunction.h"
#include <iostream>
using namespace std;

void MembershipFunction::setShapeParameters(double a, double b, double c, double d)
	{
		m_PointA = a;
		m_PointB = b;
		m_PointC = c;
		m_PointD = d;
	}

void MembershipFunction::setType(char type)
{
	m_Type = type;
}

void MembershipFunction::setName(const char* name)
{
	m_Name = new char[strlen(name) + 1];
	strcpy_s(m_Name, strlen(name) + 1, name);
}

bool MembershipFunction::isWithinRange(double t)
{
	if ((t >= m_PointA) && (t <= m_PointD)) return true; else return false;
}

char MembershipFunction::getType(void)const { return m_Type; }

void MembershipFunction::getName() const
{
	cout << m_Name << endl;
}

double MembershipFunction::calcMembershipDegree(double dist)
{
	//Calculate at a rising edge
	if (m_PointA <= dist && dist < m_PointB)
	{
		double result = (dist - m_PointA) / (m_PointB - m_PointA);
		return result;
	}
	//Return 1 if its a horizontal line
	else if (m_PointB <= dist && dist <= m_PointC)
	{
		return 1.0f;
	}
	//Calculate at falling edge
	else if (m_PointC < dist && dist <= m_PointD)
	{
		double result = (m_PointD - dist) / (m_PointD - m_PointC);
		return result;
	}
	else
	{
		return 0.0f;
	}
}

double MembershipFunction::calcMidPoint()
{
	return (m_PointB + m_PointC) / 2;
}