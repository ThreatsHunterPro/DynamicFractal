#pragma once
#include "ColorPalette.h"
#include "ViewController.h"
#include <mutex>
#include <thread>
#include <condition_variable>

class ViewRenderer
{
    int width;
    int height;
    vector<Uint8> pixels;

    ColorPalette palette;
    Texture texture;
    Sprite sprite;

    int maxIter;
    ViewController controller;

    mutex renderMutex;
    condition_variable cv;
    bool hasNewRequest;
    bool running;

    thread renderThread;

public:
    ViewRenderer(const Vector2u& _windowSize, const ColorPalette& _palette);
    ~ViewRenderer();

private:
    // Calcul d’une bande verticale (partie de l’image)
    void RenderBand(int _yStart, int _yEnd, double _centerX, double _centerY, double _zoom, int _maxIter);

public:
    // Demander un rendu (thread-safe)
    void RequestRender(const ViewController& _controller, const int _maxIter);

    void RenderLoop();

    // Faire le rendu si nécessaire (bloquant)
    void UpdateRender();

    // Dessiner le rendu
    void Draw(RenderWindow& _window);
};