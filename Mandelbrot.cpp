#include "Mandelbrot.h"

int Mandelbrot::ComputeIterations(double _x0, double _y0, int _maxIter)
{
    double _x = 0.0, _y = 0.0;
    int _iter = 0;

    while (_x * _x + _y * _y <= 4 && _iter < _maxIter)
    {
        const double _xtemp = _x * _x - _y * _y + _x0;
        _y = 2 * _x * _y + _y0;
        _x = _xtemp;
        ++_iter;
    }

    return _iter;
}