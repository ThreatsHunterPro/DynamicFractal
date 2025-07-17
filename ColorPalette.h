#pragma once
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace sf;

class ColorPalette
{
    vector<Color> colors;

public:
    // Interpole une couleur à partir d’un dégradé
    inline Color GetColor(const double _t) const 
    {
        if (_t <= 0) return colors.front();
        if (_t >= 1) return colors.back();

        // On redimensionne t sur la taille du tableau pour trouver l'intervalle de couleur
        const double _scaled = _t * (colors.size() - 1);    
        const int _i = static_cast<int>(_scaled);           // indice de la couleur de base
        const double _frac = _scaled - _i;                  // partie décimale (pour interpolation)

        // On récupère les deux couleurs entre lesquelles on doit interpoler
        const Color& _c1 = colors[_i];
        const Color& _c2 = colors[_i + 1];

        // On effectue l'interpolation linéaire canal par canal
        return Color(
            static_cast<Uint8>(_c1.r + _frac * (_c2.r - _c1.r)),
            static_cast<Uint8>(_c1.g + _frac * (_c2.g - _c1.g)),
            static_cast<Uint8>(_c1.b + _frac * (_c2.b - _c1.b))
        );
    }

public:
    ColorPalette();
};