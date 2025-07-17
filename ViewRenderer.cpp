#include "ViewRenderer.h"
#include "Mandelbrot.h"

ViewRenderer::ViewRenderer(const Vector2u& _windowSize, const ColorPalette& _palette)
{
	width = _windowSize.x;
	height = _windowSize.y;
	palette = _palette;
	pixels.resize(width * height * 4);

    // Texture
	texture.create(width, height);
	sprite.setTexture(texture);

    // Controller
    maxIter = 0;
    controller = ViewController();

    // Thread safe
    hasNewRequest = false;
    running = true;

    renderThread = thread([this]() { RenderLoop(); });
}

ViewRenderer::~ViewRenderer()
{
    {
        lock_guard<mutex> _lock(renderMutex);
        running = false;
        hasNewRequest = true;
    }

    cv.notify_one();
    renderThread.join();
}


void ViewRenderer::RequestRender(const ViewController& _controller, const int _maxIter)
{
    {
        lock_guard<mutex> _lock(renderMutex);
        controller = _controller;
        maxIter = _maxIter;
        hasNewRequest = true;
    }

    cv.notify_one();
}

void ViewRenderer::RenderLoop()
{
    while (true)
    {
        {
            unique_lock<mutex> _lock(renderMutex);
            cv.wait(_lock, [this]()
            {
                return hasNewRequest || !running;
            });
        }

        if (!running) break;

        UpdateRender();
        hasNewRequest = false;
    }
}

void ViewRenderer::UpdateRender()
{
    double _centerX, _centerY, _zoom;
    int _localMaxIter;
    {
        lock_guard<mutex> _lock(renderMutex);
        _centerX = controller.GetCenterX();
        _centerY = controller.GetCenterY();
        _zoom = controller.GetZoom();
        _localMaxIter = maxIter;
    }

    const int _threadsCount = thread::hardware_concurrency();
    const int _bandHeight = height / _threadsCount;
    vector<thread> _threads;

    for (int _index = 0; _index < _threadsCount; ++_index)
    {
        const int _yStart = _index * _bandHeight;
        const int _yEnd = (_index == _threadsCount - 1) ? height : _yStart + _bandHeight;
        _threads.emplace_back(&ViewRenderer::RenderBand, this, _yStart, _yEnd, _centerX, _centerY, _zoom, _localMaxIter);
    }

    for (thread& _thread : _threads)
    {
        _thread.join();
    }

    texture.update(pixels.data());
}

// Calcul d’une bande verticale (partie de l’image)
void ViewRenderer::RenderBand(int _yStart, int _yEnd, double _centerX, double _centerY, double _zoom, int _maxIter)
{
    for (int _pixelY = _yStart; _pixelY < _yEnd; ++_pixelY)
    {
        for (int _pixelX = 0; _pixelX < width; ++_pixelX)
        {
            const double _x0 = _centerX + (_pixelX - width / 2.0) / _zoom;
            const double _y0 = _centerY + (_pixelY - height / 2.0) / _zoom;
            const int _iter = Mandelbrot::ComputeIterations(_x0, _y0, _maxIter);
            const double _t = _maxIter > 0 ? static_cast<double>(_iter) / _maxIter : 0.0;
            const Color& _color = palette.GetColor(_t);

            const int _index = (_pixelY * width + _pixelX) * 4;
            pixels[_index + 0] = _color.r;
            pixels[_index + 1] = _color.g;
            pixels[_index + 2] = _color.b;
            pixels[_index + 3] = 255;
        }
    }
}

void ViewRenderer::Draw(RenderWindow& _window)
{
    _window.draw(sprite);
}