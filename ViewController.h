#pragma once

class ViewController
{
    double centerX, centerY;
    double zoom;

public:
    inline double GetCenterX() const 
    { 
        return centerX; 
    }
    inline double GetCenterY() const 
    { 
        return centerY; 
    }
    inline double GetZoom() const 
    { 
        return zoom; 
    }

public:
    ViewController();

public:
    void SmoothZoomAuto(const double _factor, const double _targetX, const double _targetY);
};