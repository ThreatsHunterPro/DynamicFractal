#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>

using namespace sf;
using namespace std;

Color ConvertToRGB(float H, float S, float V)
{
    float s = S / 100.0f;
    float v = V / 100.0f;
    float C = s * v;
    float X = C * (1 - fabs(fmod(H / 60.0f, 2) - 1));
    float m = v - C;

    float r, g, b;

    if (H < 60) { r = C; g = X; b = 0; }
    else if (H < 120) { r = X; g = C; b = 0; }
    else if (H < 180) { r = 0; g = C; b = X; }
    else if (H < 240) { r = 0; g = X; b = C; }
    else if (H < 300) { r = X; g = 0; b = C; }
    else { r = C; g = 0; b = X; }

    return Color((r + m) * 255, (g + m) * 255, (b + m) * 255);
}

double IterMandolbrot(double cx, double cy, int maxIter)
{
    double x = 0.0, y = 0.0;
    int iter = 0;
    while (x * x + y * y <= 4 && iter < maxIter)
    {
        double xNew = x * x - y * y + cx;
        y = 2 * x * y + cy;
        x = xNew;
        iter++;
    }
    return iter;
}

struct MandelbrotValues
{
    double xMin, xMax, yMin, yMax;
    double yRange;
};

RectangleShape InitZoom(const unsigned int _width, const unsigned int _height)
{
    float _zoomRatio = 8.0f;
    RectangleShape _zoomBorder = RectangleShape(Vector2f(_width, _height) / _zoomRatio);
    _zoomBorder.setFillColor(Color::Transparent);
    _zoomBorder.setOutlineColor(Color::White);
    _zoomBorder.setOutlineThickness(3.0f);
    _zoomBorder.setOrigin(Vector2f(_zoomBorder.getSize().x, _zoomBorder.getSize().y) / 2.0f);
    return _zoomBorder;
}

void InitDescription(RectangleShape& _background, Font& _font, Text& _zoomText, Text& _precisionText, const Vector2f& _descriptionSize = Vector2f(200.0f, 100.0f))
{
    _background = RectangleShape(_descriptionSize);
    _background.setFillColor(Color(128, 128, 128));
    _background.setOutlineColor(Color::Black);
    _background.setOutlineThickness(3.0f);
    _background.setPosition(Vector2f(10.0f, 10.0f));

    if (!_font.loadFromFile("arial.ttf"))
    {
        cerr << "Erreur => Impossible de charger la police arial.ttf !" << endl;
        return;
    }

    _zoomText.setFont(_font);
    _zoomText.setFillColor(Color::White);
    _zoomText.setCharacterSize(24);
    _zoomText.setPosition(Vector2f(20.0f, 20.0f));

    _precisionText.setFont(_font);
    _precisionText.setFillColor(Color::White);
    _precisionText.setCharacterSize(24);
    _precisionText.setPosition(Vector2f(20.0f, 60.0f));
}

MandelbrotValues InitMandelbrotValues(const unsigned int _width, const unsigned int _height)
{
    double xMin = -2.4;
    double xMax = 1.0;
    double yRange = (xMax - xMin) * _height / _width;
    double yMin = -yRange / 2.0;
    double yMax = yRange / 2.0;

    return { xMin, xMax, yMin, yMax, yRange };
}

Texture ComputeMandelbrot(const unsigned int _width, const unsigned int _height, const MandelbrotValues& _values, const int _iterationsCount)
{
    Texture _texture;
    _texture.create(_width, _height);
    Uint8* _pixels = new Uint8[_width * _height * 4];

    for (int x = 0; x < _width; x++)
    {
        for (int y = 0; y < _height; y++)
        {
            double real = _values.xMin + (_values.xMax - _values.xMin) * x / (_width - 1.0);
            double imag = _values.yMin + (_values.yMax - _values.yMin) * y / (_height - 1.0);
            double i = IterMandolbrot(real, imag, _iterationsCount);

            int pos = 4 * (y * _width + x);
            Color c = ConvertToRGB(255 * i / _iterationsCount, 100, (i < _iterationsCount) ? 100 : 0);

            _pixels[pos] = c.r;
            _pixels[pos + 1] = c.g;
            _pixels[pos + 2] = c.b;
            _pixels[pos + 3] = 255;
        }
    }

    _texture.update(_pixels);
    delete[] _pixels;
    return _texture;
}

void Update(const unsigned int _width, const unsigned int _height, const MandelbrotValues& _values, const float _precision, const int _level, Texture& _texture, Sprite& _sprite, Text& _zoomText, Text& _precisionText)
{
    _texture = ComputeMandelbrot(_width, _height, _values, (int)_precision);
    _sprite.setTexture(_texture);
    _zoomText.setString("Zoom: " + to_string(static_cast<int>(pow(8, _level - 1))));
    _precisionText.setString("Iterations : " + to_string(_level) + " / " + to_string((int)_precision));
}

float Normalize(float value, float minIn, float maxIn, float minOut, float maxOut)
{
    return minOut + (value - minIn) * (maxOut - minOut) / (maxIn - minIn);
}

MandelbrotValues Normalize(const Vector2f& min, const Vector2f& max, const unsigned int width, const unsigned int height, const MandelbrotValues& values)
{
    return {
        Normalize(min.x, 0.0f, width, values.xMin, values.xMax),
        Normalize(max.x, 0.0f, width, values.xMin, values.xMax),
        Normalize(min.y, 0.0f, height, values.yMin, values.yMax),
        Normalize(max.y, 0.0f, height, values.yMin, values.yMax),
        values.yRange
    };
}

void Fractal()
{
    unsigned int width = 1600, height = 900;
    RenderWindow window(VideoMode(width, height), "Fractale de Mandelbrot");

    RectangleShape zoomBorder = InitZoom(width, height);
    float precision = 64.0f;
    int level = 1;

    MandelbrotValues defaultValues = InitMandelbrotValues(width, height);
    MandelbrotValues values = defaultValues;

    Texture mandelbrotTexture = ComputeMandelbrot(width, height, values, (int)precision);
    Sprite mandelbrotSprite(mandelbrotTexture);

    RectangleShape descriptionBackground;
    Font font;
    Text zoomText, precisionText;
    InitDescription(descriptionBackground, font, zoomText, precisionText);

    zoomText.setString("Zoom: 1");
    precisionText.setString("Iterations : 1 / " + to_string((int)precision));

    Clock autoZoomClock;
    const float zoomSpeed = 0.95f;
    const double zoomTargetReal = -0.743643887037151;
    const double zoomTargetImag = 0.13182590420533;

    while (window.isOpen())
    {
        Event event;
        while (window.pollEvent(event))
        {
            if (event.type == Event::Closed)
                window.close();
        }

        if (autoZoomClock.getElapsedTime().asMilliseconds() > 33)
        {
            double centerX = (zoomTargetReal - values.xMin) / (values.xMax - values.xMin) * width;
            double centerY = (zoomTargetImag - values.yMin) / (values.yMax - values.yMin) * height;
            Vector2f center((float)centerX, (float)centerY);

            Vector2f halfSize(width / 2.0f, height / 2.0f);
            Vector2f zoomMin = center - halfSize * zoomSpeed;
            Vector2f zoomMax = center + halfSize * zoomSpeed;

            values = Normalize(zoomMin, zoomMax, width, height, values);
            level++;
            precision += 1.0f;

            Update(width, height, values, precision, level, mandelbrotTexture, mandelbrotSprite, zoomText, precisionText);
            autoZoomClock.restart();
        }

        window.clear();
        window.draw(mandelbrotSprite);
        window.draw(descriptionBackground);
        window.draw(zoomText);
        window.draw(precisionText);
        window.display();
    }
}

#include <SFML/Graphics.hpp>
#include <vector>
#include <stack>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <set>

struct Cell {
    bool visited = false;
    bool walls[4] = { true, true, true, true }; // haut, droite, bas, gauche
};

const int COLS = 20;
const int ROWS = 15;
const int CELL_SIZE = 30;

int index(int x, int y) {
    if (x < 0 || y < 0 || x >= COLS || y >= ROWS)
        return -1;
    return x + y * COLS;
}

class Maze {
public:
    std::vector<Cell> grid;

    Maze() : grid(COLS* ROWS) {}

    void removeWalls(int current, int next) {
        int x1 = current % COLS;
        int y1 = current / COLS;
        int x2 = next % COLS;
        int y2 = next / COLS;

        if (x1 == x2) {
            if (y1 > y2) {
                grid[current].walls[0] = false; // mur haut
                grid[next].walls[2] = false;    // mur bas
            }
            else {
                grid[current].walls[2] = false;
                grid[next].walls[0] = false;
            }
        }
        else if (y1 == y2) {
            if (x1 > x2) {
                grid[current].walls[3] = false; // mur gauche
                grid[next].walls[1] = false;    // mur droite
            }
            else {
                grid[current].walls[1] = false;
                grid[next].walls[3] = false;
            }
        }
    }

    std::vector<int> getUnvisitedNeighbors(int x, int y) {
        std::vector<int> neighbors;
        if (index(x, y - 1) != -1 && !grid[index(x, y - 1)].visited)
            neighbors.push_back(index(x, y - 1));
        if (index(x + 1, y) != -1 && !grid[index(x + 1, y)].visited)
            neighbors.push_back(index(x + 1, y));
        if (index(x, y + 1) != -1 && !grid[index(x, y + 1)].visited)
            neighbors.push_back(index(x, y + 1));
        if (index(x - 1, y) != -1 && !grid[index(x - 1, y)].visited)
            neighbors.push_back(index(x - 1, y));
        return neighbors;
    }

    // DFS classique (backtracking)
    void generateDFS() {
        for (auto& cell : grid) cell.visited = false;
        std::stack<int> stack;
        int current = 0;
        grid[current].visited = true;

        while (true) {
            int x = current % COLS;
            int y = current / COLS;
            auto neighbors = getUnvisitedNeighbors(x, y);

            if (!neighbors.empty()) {
                int next = neighbors[rand() % neighbors.size()];
                removeWalls(current, next);
                stack.push(current);
                current = next;
                grid[current].visited = true;
            }
            else if (!stack.empty()) {
                current = stack.top();
                stack.pop();
            }
            else {
                break;
            }
        }
    }

    // Algorithme Prim version labyrinthe
    void generatePrim() {
        for (auto& cell : grid) cell.visited = false;

        std::vector<int> walls; // murs entre cellules (représentés par la cellule adjacente)
        int start = 0;
        grid[start].visited = true;

        auto addWalls = [&](int cellIdx) {
            int x = cellIdx % COLS;
            int y = cellIdx / COLS;
            int n;
            // haut
            n = index(x, y - 1);
            if (n != -1 && !grid[n].visited) walls.push_back(n);
            // droite
            n = index(x + 1, y);
            if (n != -1 && !grid[n].visited) walls.push_back(n);
            // bas
            n = index(x, y + 1);
            if (n != -1 && !grid[n].visited) walls.push_back(n);
            // gauche
            n = index(x - 1, y);
            if (n != -1 && !grid[n].visited) walls.push_back(n);
            };

        addWalls(start);

        while (!walls.empty()) {
            int randIndex = rand() % walls.size();
            int cellIdx = walls[randIndex];

            // Trouver les voisins visités
            int x = cellIdx % COLS;
            int y = cellIdx / COLS;

            std::vector<int> visitedNeighbors;
            int n;

            n = index(x, y - 1);
            if (n != -1 && grid[n].visited) visitedNeighbors.push_back(n);
            n = index(x + 1, y);
            if (n != -1 && grid[n].visited) visitedNeighbors.push_back(n);
            n = index(x, y + 1);
            if (n != -1 && grid[n].visited) visitedNeighbors.push_back(n);
            n = index(x - 1, y);
            if (n != -1 && grid[n].visited) visitedNeighbors.push_back(n);

            if (!visitedNeighbors.empty()) {
                int neighbor = visitedNeighbors[rand() % visitedNeighbors.size()];
                removeWalls(cellIdx, neighbor);
                grid[cellIdx].visited = true;

                addWalls(cellIdx);
            }

            walls.erase(walls.begin() + randIndex);
        }
    }

    void draw(sf::RenderWindow& window) {
        for (int y = 0; y < ROWS; y++) {
            for (int x = 0; x < COLS; x++) {
                int i = index(x, y);
                sf::Vector2f pos(x * CELL_SIZE, y * CELL_SIZE);

                sf::Vertex line[2];

                if (grid[i].walls[0]) { // haut
                    line[0] = sf::Vertex(pos, sf::Color::Black);
                    line[1] = sf::Vertex(sf::Vector2f(pos.x + CELL_SIZE, pos.y), sf::Color::Black);
                    window.draw(line, 2, sf::Lines);
                }
                if (grid[i].walls[1]) { // droite
                    line[0] = sf::Vertex(sf::Vector2f(pos.x + CELL_SIZE, pos.y), sf::Color::Black);
                    line[1] = sf::Vertex(sf::Vector2f(pos.x + CELL_SIZE, pos.y + CELL_SIZE), sf::Color::Black);
                    window.draw(line, 2, sf::Lines);
                }
                if (grid[i].walls[2]) { // bas
                    line[0] = sf::Vertex(sf::Vector2f(pos.x + CELL_SIZE, pos.y + CELL_SIZE), sf::Color::Black);
                    line[1] = sf::Vertex(sf::Vector2f(pos.x, pos.y + CELL_SIZE), sf::Color::Black);
                    window.draw(line, 2, sf::Lines);
                }
                if (grid[i].walls[3]) { // gauche
                    line[0] = sf::Vertex(sf::Vector2f(pos.x, pos.y + CELL_SIZE), sf::Color::Black);
                    line[1] = sf::Vertex(pos, sf::Color::Black);
                    window.draw(line, 2, sf::Lines);
                }
            }
        }
    }
};

void ExecuteMaze() {
    srand(static_cast<unsigned int>(time(nullptr)));

    Maze maze;

    std::cout << "Choisissez l'algorithme de generation de labyrinthe :\n";
    std::cout << "1 - DFS (Backtracking)\n";
    std::cout << "2 - Prim\n";
    int choice;
    std::cin >> choice;

    if (choice == 1) {
        maze.generateDFS();
    }
    else if (choice == 2) {
        maze.generatePrim();
    }
    else {
        std::cout << "Choix invalide. Generation par DFS par defaut.\n";
        maze.generateDFS();
    }

    sf::RenderWindow window(sf::VideoMode(COLS * CELL_SIZE, ROWS * CELL_SIZE), "Labyrinthe SFML - Choix algo");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        maze.draw(window);
        window.display();
    }
}


int main()
{
    //Fractal();
    ExecuteMaze();

    return 0;
}