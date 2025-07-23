#include "ViewRenderer.h"

using namespace sf;
using namespace std;

//int main()
//{
//    const Vector2u& _windowSize = Vector2u(600, 400);
//    RenderWindow _window = RenderWindow(VideoMode(_windowSize.x, _windowSize.y), "Dynamic Fractal");
//    _window.setFramerateLimit(60);
//
//    ColorPalette _palette;
//    ViewRenderer _renderer = ViewRenderer(_windowSize, _palette);
//    ViewController _controller;
//
//    // Variables FPS
//    Clock _clock;
//    float _fps = 0.0f;
//    float _fpsUpdateTimer = 0.0f;
//
//    constexpr double _zoomFactor = 1.01;
//    // Seahorse Valley	    (-0.74543, 0.11301)
//    // Elephant Valley      (-0.745, 0.105)
//    // Triple Spiral	    (-0.088, 0.654)
//    // Mini Mandelbrot	    (-1.749, 0.0)
//    // Needle Point Zoom	(-0.743643887037151, 0.13182590420533)
//    Vector2<double> _targetLocation = Vector2<double>(-0.7454282500, 0.1130090000);
//    int _maxIter = 300;
//
//    //V2
//    const double _baseZoom = _controller.GetZoom();
//    const int _baseIter = 30;
//    const int _maxCap = 300;
//
//    Font _font;
//    if (!_font.loadFromFile("arial.ttf"))
//    {
//        cerr << "Erreur : police 'arial.ttf' inaccessible !" << endl;
//    }
//
//    Text _fpsText = Text("...", _font, 18);
//    _fpsText.setFillColor(Color::White);
//
//    while (_window.isOpen())
//    {
//        Event _event;
//        while (_window.pollEvent(_event)) 
//        {
//            if (_event.type == Event::Closed)
//            {
//                _window.close();
//                continue;
//            }
//        }
//
//        const float _deltaTime = _clock.restart().asSeconds();
//        _fpsUpdateTimer += _deltaTime;
//        _fps = 1.0f / _deltaTime;
//
//        if (_fpsUpdateTimer >= 0.5f)
//        {
//            _fpsText.setString(to_string(static_cast<int>(_fps)) + " FPS");
//            _fpsText.setPosition(_windowSize.x - _fpsText.getLocalBounds().width - 10.0f, 10.0f);
//            _fpsUpdateTimer = 0.0f;
//        }
//
//        // Zoom automatique
//        _controller.SmoothZoomAuto(_zoomFactor, _targetLocation.x, _targetLocation.y);
//
//        _renderer.RequestRender(_controller, _maxIter);
//        const double _currentZoom = _controller.GetZoom();
//
//        //V1
//        //_maxIter = static_cast<int>(50 * log2(_currentZoom / 300));
//
//        //V2
//        //const double _zoomRatio = _currentZoom / _baseZoom;
//        //_maxIter = static_cast<int>(_baseIter + 5 * log2(_zoomFactor + 1.0));
//        //_maxIter = static_cast<int>(300 * pow(_zoomRatio, 0.3));
//        //_maxIter = static_cast<int>(_maxCap - (_maxCap + _baseIter) / (_zoomRatio + 1.0)); // asymptote
//
//        //V3
//        const double _zoomRatio = _currentZoom / _baseZoom;
//        const double _dynamicCap = _baseIter * log2(_zoomRatio + 1.0);
//        _maxIter = static_cast<int>(_dynamicCap - (_dynamicCap - _baseIter) / (_zoomRatio + 1.0));
//
//        //V4
//        //const double zoomRatio = std::max(_currentZoom / _baseZoom, 1.0);
//        //const int minIter = _baseIter;
//        //const int maxAllowedIter = 3000; // ou 3000 si ta machine le supporte
//        //// Courbe douce avec racine
//        //const double growth = sqrt(log2(zoomRatio + 1.0));
//        //const int computedIter = static_cast<int>(minIter + 400.0 * growth);
//        //// Clamp entre base et max autorisé
//        //_maxIter = std::min(computedIter, maxAllowedIter);
//
//        cout << _maxIter << endl;
//
//        _window.clear();
//        _renderer.Draw(_window);
//        _window.draw(_fpsText);
//        _window.display();
//    }
//
//	return EXIT_SUCCESS;
//}

int main()
{
    const Vector2u& _windowSize = Vector2u(1280, 720);
    RenderWindow _window(VideoMode(_windowSize.x, _windowSize.y), "Dynamic Fractal");
    _window.setFramerateLimit(60);

    ColorPalette _palette;
    ViewRenderer _renderer = ViewRenderer(_windowSize, _palette);
    ViewController _controller;

    // Variables FPS
    Clock _clock;
    float _fps = 0.0f;
    float _fpsUpdateTimer = 0.0f;

    constexpr double _zoomFactor = 1.03;
    Vector2<double> _targetLocation = Vector2<double>(-0.74542825, 0.11300875);
    int _maxIter = 300;

    // Paramètres dynamiques d'itération
    const double _baseZoom = _controller.GetZoom();
    const int _baseIter = 30;
    const int _maxAllowedIter = 3000;

    // Capture
    int _frameNumber = 0;
    const int _maxFrames = 600; // environ 10s à 60 fps

    while (_window.isOpen() && _frameNumber < _maxFrames)
    {
        Event _event;
        while (_window.pollEvent(_event))
        {
            if (_event.type == Event::Closed)
            {
                _window.close();
                continue;
            }
        }

        // Zoom automatique
        _controller.SmoothZoomAuto(_zoomFactor, _targetLocation.x, _targetLocation.y);

        const double _currentZoom = _controller.GetZoom();
        const double zoomRatio = std::max(_currentZoom / _baseZoom, 1.0);
        const double growth = sqrt(log2(zoomRatio + 1.0));
        const int computedIter = static_cast<int>(_baseIter + 400.0 * growth);
        _maxIter = std::min(computedIter, _maxAllowedIter);

        _renderer.RequestRender(_controller, _maxIter);

        _window.clear();
        _renderer.Draw(_window);
        _window.display();

        // Capture de la frame
        Texture texture;
        texture.create(_windowSize.x, _windowSize.y);
        texture.update(_window);
        Image screenshot = texture.copyToImage();

        string filename = "Frames/Mandelbrot/frame_" + to_string(_frameNumber++) + ".png";
        screenshot.saveToFile(filename);

        cout << "Frame " << _frameNumber << " saved. Iter = " << _maxIter << endl;
    }

    return EXIT_SUCCESS;
}
