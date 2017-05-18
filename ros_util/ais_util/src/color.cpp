/*
 * This file (color.cpp) is part of the "ais_definitions" packages of Daniel Kuhner.
 *
 * It is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
 *
 * created:		Mar 13, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */

#include <ais_definitions/exception.h>
#include <ais_util/color.h>
#include <cmath>

using namespace ais_definitions;

namespace ais_util
{

Color::Color() :
				m_r(1),
				m_g(1),
				m_b(1),
				m_a(1)
{
}

Color::Color(double gray,
		double alpha) :
				m_r(gray),
				m_g(gray),
				m_b(gray),
				m_a(alpha)
{
}

Color::Color(const double r,
		const double g,
		const double b,
		const double a) :
				m_r(r),
				m_g(g),
				m_b(b),
				m_a(a)
{
}

Color::Color(const double h,
		const double s,
		const double v)
{
	fromHSV(h, s, v);
}

Color::Color(const ColorCode color)
{
	setColor(color);
}

double& Color::r()
{
	return m_r;
}

double& Color::g()
{
	return m_g;
}

double& Color::b()
{
	return m_b;
}

double& Color::a()
{
	return m_a;
}

double Color::r() const
{
	return m_r;
}

double Color::g() const
{
	return m_g;
}

double Color::b() const
{
	return m_b;
}

double Color::a() const
{
	return m_a;
}

unsigned char Color::rInt() const
{
	return m_r * 255;
}

unsigned char Color::gInt() const
{
	return m_g * 255;
}

unsigned char Color::bInt() const
{
	return m_b * 255;
}

unsigned char Color::aInt() const
{
	return m_a * 255;
}

void Color::setColor(const ColorCode color)
{
	switch (color)
	{
		case Red:
			setColor(1.0, 0.0, 0.0, 1.0);
			break;
		case Red50:
			setColor(0.5, 0.0, 0.0, 1.0);
			break;
		case Blue:
			setColor(0.0, 0.0, 1.0, 1.0);
			break;
		case Blue50:
			setColor(0.0, 0.0, 0.5, 1.0);
			break;
		case Green:
			setColor(0.0, 1.0, 0.0, 1.0);
			break;
		case Green50:
			setColor(0.0, 0.5, 0.0, 1.0);
			break;
		case Yellow:
			setColor(1.0, 1.0, 0.0, 1.0);
			break;
		case Orange:
			setColor(1.0, 0.5, 0.0, 1.0);
			break;
		case Cyan:
			setColor(0.0, 1.0, 1.0, 1.0);
			break;
		case Magenta:
			setColor(1.0, 0.0, 1.0, 1.0);
			break;
		case Purple:
			setColor(0.63, 0.13, 0.94, 1.0);
			break;
		case White:
			setColor(1.0, 1.0, 1.0, 1.0);
			break;
		case Gray:
			setColor(0.6, 0.6, 0.6, 1.0);
			break;
		default:
			setColor(0.0, 0.0, 0.0, 1.0);
			break;
	}
}

void Color::setColor(const double r,
		const double g,
		const double b,
		const double a)
{
	m_r = r;
	m_g = g;
	m_b = b;
	m_a = a;
}

Color Color::fromInt(const int r,
		const int g,
		const int b,
		const int a)
{
	return Color(r / 255.0, g / 255.0, b / 255.0, a / 255.0);
}

Color Color::fromHSV(const double h,
		const double s,
		const double v)
{
	if (s < 0.0 || s > 1.0)
	{
		throw Exception("saturation not in range [0, 1]");
	}
	if (v < 0.0 || v > 1.0)
	{
		throw Exception("value not in range [0, 1]");
	}
	const double c = v * s;
	const double x = c * (1 - fabs(fmod(h / 60, 2) - 1));
	const double m = v - c;

	double r_, g_, b_;
	if (h < 0)
	{
		throw Exception("hue not in range [0, 360)");
	}
	else if (h < 60)
	{
		r_ = c;
		g_ = x;
		b_ = 0;
	}
	else if (h < 120)
	{
		r_ = x;
		g_ = c;
		b_ = 0;
	}
	else if (h < 180)
	{
		r_ = 0;
		g_ = c;
		b_ = x;
	}
	else if (h < 240)
	{
		r_ = 0;
		g_ = x;
		b_ = c;
	}
	else if (h < 300)
	{
		r_ = x;
		g_ = 0;
		b_ = c;
	}
	else if (h < 360)
	{
		r_ = c;
		g_ = 0;
		b_ = x;
	}
	else
	{
		throw Exception("hue not in range [0, 360)");
	}

	return Color(r_ + m, g_ + m, b_ + m, 1.0);
}

Color Color::random()
{
	return Color((rand() % 1000) / 1000.0, (rand() % 1000) / 1000.0, (rand() % 1000) / 1000.0, 1);
}

Color Color::scale(const double value,
		const ColorScale type)
{
	if (value < 0.0 || value > 1.0)
	{
		throw Exception("value must be in range [0, 1]");
	}

	Color color;

	switch (type)
	{
		case ColorScaleRedGreen:
			color = fromHSV(value * 120.0, 1.0, 1.0);
			break;
		case ColorScaleRainbow:
			color = fromHSV(value * 270.0, 1.0, 1.0);
			break;
		case ColorScaleGray:
			color = Color(value);
			break;
		default:
			throw Exception("Unknown color scale");
	}

	return color;
}

Color ais_util::Color::red()
{
	return Color(Red);
}

Color ais_util::Color::blue()
{
	return Color(Blue);
}

Color Color::green()
{
	return Color(Green);
}

Color Color::yellow()
{
	return Color(Yellow);
}

Color Color::orange()
{
	return Color(Orange);
}

Color Color::white()
{
	return Color(White);
}

Color Color::black()
{
	return Color(Black);
}

std_msgs::ColorRGBA Color::toROSMsg()
{
	std_msgs::ColorRGBA c;
	c.a = m_a;
	c.r = m_r;
	c.g = m_g;
	c.b = m_b;
	return c;
}

}

std::ostream& operator <<(std::ostream& stream,
		const ais_util::Color& color)
{
	stream << "Color: rgb [" << (int) color.rInt() << " " << (int) color.gInt() << " " << (int) color.bInt() << "]";
	return stream;
}

