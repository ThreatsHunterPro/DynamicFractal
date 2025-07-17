#include "ViewController.h"

ViewController::ViewController()
{
	centerX = -0.75;
	centerY = 0.1;
	zoom = 300;
}


void ViewController::SmoothZoomAuto(const double _factor, const double _targetX, const double _targetY)
{
	const double _newZoom = zoom * _factor;
	centerX = _targetX - (_targetX - centerX) * (zoom / _newZoom);
	centerY = _targetY - (_targetY - centerY) * (zoom / _newZoom);
	zoom = _newZoom;
}