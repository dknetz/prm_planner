///*
// * This file (colors.h) is part of the "ais_definitions" packages of Daniel Kuhner.
// *
// * It is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * It is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
// *
// * created:		Dec 19, 2013
// * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
// *
// */
#ifndef ROBOTIC_LIBS_UTIL_COLORS_H_
#define ROBOTIC_LIBS_UTIL_COLORS_H_

#include <ais_definitions/class.h>
#include <ostream>
#include <std_msgs/ColorRGBA.h>

namespace ais_util
{

class Color
{
PUBLIC_ENUMS:
	enum ColorCode
	{
		Red,
		Red50,
		Blue,
		Blue50,
		Green,
		Green50,
		Yellow,
		Orange,
		Cyan,
		Magenta,
		Purple,
		White,
		Black,
		Gray
	};

	enum ColorScale
	{
		ColorScaleRedGreen,
		ColorScaleRainbow,
		ColorScaleGray
	};

PUBLIC_METHODS:
	//Default: Constructs white with alpha = 1
	Color();

	//Gray color
	Color(double gray,
			double alpha = 1.0);

	//RGBA between 0 and 1
	Color(const double r,
			const double g,
			const double b,
			const double a = 1.0);

	//HSV
	Color(const double h,
			const double s,
			const double v);

	//Color code
	Color(const ColorCode color);

	double& r();
	double& g();
	double& b();
	double& a();

	double r() const;
	double g() const;
	double b() const;
	double a() const;

	unsigned char rInt() const;
	unsigned char gInt() const;
	unsigned char bInt() const;
	unsigned char aInt() const;

	void setColor(const double r,
			const double g,
			const double b,
			const double a = 1.0);

	void setColor(const ColorCode color);

	std_msgs::ColorRGBA toROSMsg();

	static Color fromInt(const int r,
			const int g,
			const int b,
			const int a = 255);
	static Color fromHSV(const double h,
			const double s,
			const double v);
	static Color random();
	static Color scale(const double value,
			const ColorScale type);

	static Color red();
	static Color blue();
	static Color green();
	static Color yellow();
	static Color orange();
	static Color white();
	static Color black();

PRIVATE_VARIABLES:
	double m_r;
	double m_g;
	double m_b;
	double m_a;
};

}

std::ostream& operator<< (std::ostream& stream, const ais_util::Color& color);

#endif /* ROBOTIC_LIBS_UTIL_COLORS_H_ */
