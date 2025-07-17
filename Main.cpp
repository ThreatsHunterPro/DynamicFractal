#include "ViewRenderer.h"

using namespace sf;
using namespace std;

int main()
{
    const Vector2u& _windowSize = Vector2u(600, 400);
    RenderWindow _window = RenderWindow(VideoMode(_windowSize.x, _windowSize.y), "Dynamic Fractal");
    _window.setFramerateLimit(60);

    ColorPalette _palette;
    ViewRenderer _renderer = ViewRenderer(_windowSize, _palette);
    ViewController _controller;

    // Variables FPS
    Clock _clock;
    float _fps = 0.0f;
    float _fpsUpdateTimer = 0.0f;

    constexpr double _zoomFactor = 1.03;
    Vector2<double> _targetLocation = Vector2<double>(-0.74543, 0.11301);
    int _maxIter = 300;

    Font _font;
    if (!_font.loadFromFile("arial.ttf"))
    {
        cerr << "Erreur : police 'arial.ttf' inaccessible !" << endl;
    }

    Text _fpsText = Text("...", _font, 18);
    _fpsText.setFillColor(Color::White);

    while (_window.isOpen())
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

        const float _deltaTime = _clock.restart().asSeconds();
        _fpsUpdateTimer += _deltaTime;
        _fps = 1.0f / _deltaTime;

        if (_fpsUpdateTimer >= 0.5f)
        {
            _fpsText.setString(to_string(static_cast<int>(_fps)) + " FPS");
            _fpsText.setPosition(_windowSize.x - _fpsText.getLocalBounds().width - 10.0f, 10.0f);
            _fpsUpdateTimer = 0.0f;
        }

        // Zoom automatique
        _controller.SmoothZoomAuto(_zoomFactor, _targetLocation.x, _targetLocation.y);

        _renderer.RequestRender(_controller, _maxIter);

        _window.clear();
        _renderer.Draw(_window);
        _window.draw(_fpsText);
        _window.display();
    }

	return EXIT_SUCCESS;
}