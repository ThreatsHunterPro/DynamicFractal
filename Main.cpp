#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>

using namespace sf;
using namespace std;

//Color ConvertToRGB(float H, float S, float V)
//{
//    float s = S / 100.0f;
//    float v = V / 100.0f;
//    float C = s * v;
//    float X = C * (1 - fabs(fmod(H / 60.0f, 2) - 1));
//    float m = v - C;
//
//    float r, g, b;
//
//    if (H < 60) { r = C; g = X; b = 0; }
//    else if (H < 120) { r = X; g = C; b = 0; }
//    else if (H < 180) { r = 0; g = C; b = X; }
//    else if (H < 240) { r = 0; g = X; b = C; }
//    else if (H < 300) { r = X; g = 0; b = C; }
//    else { r = C; g = 0; b = X; }
//
//    return Color((r + m) * 255, (g + m) * 255, (b + m) * 255);
//}
//
//double IterMandolbrot(double cx, double cy, int maxIter)
//{
//    double x = 0.0, y = 0.0;
//    int iter = 0;
//    while (x * x + y * y <= 4 && iter < maxIter)
//    {
//        double xNew = x * x - y * y + cx;
//        y = 2 * x * y + cy;
//        x = xNew;
//        iter++;
//    }
//    return iter;
//}
//
//struct MandelbrotValues
//{
//    double xMin, xMax, yMin, yMax;
//    double yRange;
//};
//
//RectangleShape InitZoom(const unsigned int _width, const unsigned int _height)
//{
//    float _zoomRatio = 8.0f;
//    RectangleShape _zoomBorder = RectangleShape(Vector2f(_width, _height) / _zoomRatio);
//    _zoomBorder.setFillColor(Color::Transparent);
//    _zoomBorder.setOutlineColor(Color::White);
//    _zoomBorder.setOutlineThickness(3.0f);
//    _zoomBorder.setOrigin(Vector2f(_zoomBorder.getSize().x, _zoomBorder.getSize().y) / 2.0f);
//    return _zoomBorder;
//}
//
//void InitDescription(RectangleShape& _background, Font& _font, Text& _zoomText, Text& _precisionText, const Vector2f& _descriptionSize = Vector2f(200.0f, 100.0f))
//{
//    _background = RectangleShape(_descriptionSize);
//    _background.setFillColor(Color(128, 128, 128));
//    _background.setOutlineColor(Color::Black);
//    _background.setOutlineThickness(3.0f);
//    _background.setPosition(Vector2f(10.0f, 10.0f));
//
//    if (!_font.loadFromFile("arial.ttf"))
//    {
//        cerr << "Erreur => Impossible de charger la police arial.ttf !" << endl;
//        return;
//    }
//
//    _zoomText.setFont(_font);
//    _zoomText.setFillColor(Color::White);
//    _zoomText.setCharacterSize(24);
//    _zoomText.setPosition(Vector2f(20.0f, 20.0f));
//
//    _precisionText.setFont(_font);
//    _precisionText.setFillColor(Color::White);
//    _precisionText.setCharacterSize(24);
//    _precisionText.setPosition(Vector2f(20.0f, 60.0f));
//}
//
//MandelbrotValues InitMandelbrotValues(const unsigned int _width, const unsigned int _height)
//{
//    double xMin = -2.4;
//    double xMax = 1.0;
//    double yRange = (xMax - xMin) * _height / _width;
//    double yMin = -yRange / 2.0;
//    double yMax = yRange / 2.0;
//
//    return { xMin, xMax, yMin, yMax, yRange };
//}
//
//Texture ComputeMandelbrot(const unsigned int _width, const unsigned int _height, const MandelbrotValues& _values, const int _iterationsCount)
//{
//    Texture _texture;
//    _texture.create(_width, _height);
//    Uint8* _pixels = new Uint8[_width * _height * 4];
//
//    for (int x = 0; x < _width; x++)
//    {
//        for (int y = 0; y < _height; y++)
//        {
//            double real = _values.xMin + (_values.xMax - _values.xMin) * x / (_width - 1.0);
//            double imag = _values.yMin + (_values.yMax - _values.yMin) * y / (_height - 1.0);
//            double i = IterMandolbrot(real, imag, _iterationsCount);
//
//            int pos = 4 * (y * _width + x);
//            Color c = ConvertToRGB(255 * i / _iterationsCount, 100, (i < _iterationsCount) ? 100 : 0);
//
//            _pixels[pos] = c.r;
//            _pixels[pos + 1] = c.g;
//            _pixels[pos + 2] = c.b;
//            _pixels[pos + 3] = 255;
//        }
//    }
//
//    _texture.update(_pixels);
//    delete[] _pixels;
//    return _texture;
//}
//
//void Update(const unsigned int _width, const unsigned int _height, const MandelbrotValues& _values, const float _precision, const int _level, Texture& _texture, Sprite& _sprite, Text& _zoomText, Text& _precisionText)
//{
//    _texture = ComputeMandelbrot(_width, _height, _values, (int)_precision);
//    _sprite.setTexture(_texture);
//    _zoomText.setString("Zoom: " + to_string(static_cast<int>(pow(8, _level - 1))));
//    _precisionText.setString("Iterations : " + to_string(_level) + " / " + to_string((int)_precision));
//}
//
//float Normalize(float value, float minIn, float maxIn, float minOut, float maxOut)
//{
//    return minOut + (value - minIn) * (maxOut - minOut) / (maxIn - minIn);
//}
//
//MandelbrotValues Normalize(const Vector2f& min, const Vector2f& max, const unsigned int width, const unsigned int height, const MandelbrotValues& values)
//{
//    return {
//        Normalize(min.x, 0.0f, width, values.xMin, values.xMax),
//        Normalize(max.x, 0.0f, width, values.xMin, values.xMax),
//        Normalize(min.y, 0.0f, height, values.yMin, values.yMax),
//        Normalize(max.y, 0.0f, height, values.yMin, values.yMax),
//        values.yRange
//    };
//}
//
//void Fractal()
//{
//    unsigned int width = 1600, height = 900;
//    RenderWindow window(VideoMode(width, height), "Fractale de Mandelbrot");
//
//    RectangleShape zoomBorder = InitZoom(width, height);
//    float precision = 64.0f;
//    int level = 1;
//
//    MandelbrotValues defaultValues = InitMandelbrotValues(width, height);
//    MandelbrotValues values = defaultValues;
//
//    Texture mandelbrotTexture = ComputeMandelbrot(width, height, values, (int)precision);
//    Sprite mandelbrotSprite(mandelbrotTexture);
//
//    RectangleShape descriptionBackground;
//    Font font;
//    Text zoomText, precisionText;
//    InitDescription(descriptionBackground, font, zoomText, precisionText);
//
//    zoomText.setString("Zoom: 1");
//    precisionText.setString("Iterations : 1 / " + to_string((int)precision));
//
//    Clock autoZoomClock;
//    const float zoomSpeed = 0.95f;
//    const double zoomTargetReal = -0.743643887037151;
//    const double zoomTargetImag = 0.13182590420533;
//
//    while (window.isOpen())
//    {
//        Event event;
//        while (window.pollEvent(event))
//        {
//            if (event.type == Event::Closed)
//                window.close();
//        }
//
//        if (autoZoomClock.getElapsedTime().asMilliseconds() > 33)
//        {
//            double centerX = (zoomTargetReal - values.xMin) / (values.xMax - values.xMin) * width;
//            double centerY = (zoomTargetImag - values.yMin) / (values.yMax - values.yMin) * height;
//            Vector2f center((float)centerX, (float)centerY);
//
//            Vector2f halfSize(width / 2.0f, height / 2.0f);
//            Vector2f zoomMin = center - halfSize * zoomSpeed;
//            Vector2f zoomMax = center + halfSize * zoomSpeed;
//
//            values = Normalize(zoomMin, zoomMax, width, height, values);
//            level++;
//            precision += 1.0f;
//
//            Update(width, height, values, precision, level, mandelbrotTexture, mandelbrotSprite, zoomText, precisionText);
//            autoZoomClock.restart();
//        }
//
//        window.clear();
//        window.draw(mandelbrotSprite);
//        window.draw(descriptionBackground);
//        window.draw(zoomText);
//        window.draw(precisionText);
//        window.display();
//    }
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//#include <SFML/Graphics.hpp>
//#include <thread>
//#include <mutex>
//#include <condition_variable>
//#include <optional>
//#include <atomic>
//#include <vector>
//#include <cmath>
//
//struct MandelbrotValues2 {
//    double xMin = -2.0, xMax = 1.0;
//    double yMin = -1.5, yMax = 1.5;
//};
//
//float IterMandolbrot2(double real, double imag, int maxIter) {
//    double zReal = 0, zImag = 0;
//    int i = 0;
//    while (zReal * zReal + zImag * zImag <= 4.0 && i < maxIter) {
//        double temp = zReal * zReal - zImag * zImag + real;
//        zImag = 2.0 * zReal * zImag + imag;
//        zReal = temp;
//        ++i;
//    }
//    return static_cast<float>(i);
//}
//
//sf::Color ConvertToRGB2(float t, float s = 100, float v = 100) {
//    int i = static_cast<int>(t / 60.f) % 6;
//    float f = t / 60.f - i;
//    float p = v * (1 - s / 100.f);
//    float q = v * (1 - f * s / 100.f);
//    float r = v * (1 - (1 - f) * s / 100.f);
//    float R = 0, G = 0, B = 0;
//    switch (i) {
//    case 0: R = v; G = r; B = p; break;
//    case 1: R = q; G = v; B = p; break;
//    case 2: R = p; G = v; B = r; break;
//    case 3: R = p; G = q; B = v; break;
//    case 4: R = r; G = p; B = v; break;
//    case 5: R = v; G = p; B = q; break;
//    }
//    return sf::Color(static_cast<sf::Uint8>(R / 100.f * 255),
//        static_cast<sf::Uint8>(G / 100.f * 255),
//        static_cast<sf::Uint8>(B / 100.f * 255));
//}
//
//void BuildPixels(const MandelbrotValues2& vals, int iterations,
//    unsigned w, unsigned h, std::vector<sf::Uint8>& out)
//{
//    out.resize(w * h * 4);
//    for (unsigned x = 0; x < w; ++x) {
//        for (unsigned y = 0; y < h; ++y) {
//            double real = vals.xMin + (vals.xMax - vals.xMin) * x / (w - 1.0);
//            double imag = vals.yMin + (vals.yMax - vals.yMin) * y / (h - 1.0);
//            float i = IterMandolbrot2(real, imag, iterations);
//
//            sf::Color c = ConvertToRGB2(255 * i / iterations, 100,
//                (i < iterations) ? 100 : 0);
//
//            size_t p = 4ull * (y * w + x);
//            out[p] = c.r;
//            out[p + 1] = c.g;
//            out[p + 2] = c.b;
//            out[p + 3] = 255;
//        }
//    }
//}
//
//struct Job {
//    MandelbrotValues2 vals;
//    int iterations;
//
//    Job() = default;
//    Job(const MandelbrotValues2& _vals, const int _iterations)
//    {
//        vals = _vals;
//        iterations = _iterations;
//    }
//};
//
//std::mutex mtx;
//std::condition_variable cv;
//std::optional<Job> pending;
//std::vector<sf::Uint8> latestPixels;
//std::atomic<bool> newPixelsReady{ false };
//std::atomic<bool> running{ true };
//
//void Worker(unsigned w, unsigned h)
//{
//    std::vector<sf::Uint8> local;
//    while (running)
//    {
//        Job job;
//        {
//            std::unique_lock lk(mtx);
//            cv.wait(lk, [] { return !running || pending.has_value(); });
//            if (!running) break;
//            job = *pending;
//            pending.reset();
//        }
//
//        BuildPixels(job.vals, job.iterations, w, h, local);
//
//        {
//            std::lock_guard lk(mtx);
//            latestPixels = std::move(local);
//            newPixelsReady = true;
//        }
//    }
//}
//
//void DynamicFractal()
//{
//    const unsigned width = 800, height = 600;
//    sf::RenderWindow window(sf::VideoMode(width, height), "Mandelbrot");
//    window.setFramerateLimit(60);
//
//    sf::Texture mandelbrotTexture;
//    mandelbrotTexture.create(width, height);
//    sf::Sprite mandelbrotSprite(mandelbrotTexture);
//
//    MandelbrotValues2 values;
//    float level = 1.0f;
//    int precision = 100;
//    float currentScale = 1.0f;
//
//    std::thread worker(Worker, width, height);
//
//    while (window.isOpen()) {
//        sf::Event event;
//        while (window.pollEvent(event)) {
//            if (event.type == sf::Event::Closed)
//                window.close();
//        }
//
//        // Exemple simple de zoom auto
//        level += 0.01f;
//        double zoomFactor = std::pow(0.98, level);
//        values.xMin = -2.0 * zoomFactor;
//        values.xMax = 1.0 * zoomFactor;
//        values.yMin = -1.5 * zoomFactor;
//        values.yMax = 1.5 * zoomFactor;
//        precision = 50 + static_cast<int>(level * 2);
//
//        // Envoie une nouvelle tâche si aucune en cours
//        {
//            std::lock_guard lk(mtx);
//            if (!pending.has_value()) {
//                pending = Job(values, precision);
//                cv.notify_one();
//            }
//        }
//
//        // Met à jour la texture si un nouveau calcul est prêt
//        if (newPixelsReady.exchange(false)) {
//            std::vector<sf::Uint8> localCopy;
//            {
//                std::lock_guard lk(mtx);
//                localCopy.swap(latestPixels);
//            }
//            mandelbrotTexture.update(localCopy.data());
//        }
//
//        // Zoom visuel fluide
//        float targetScale = std::pow(8.f, level - 1);
//        currentScale += (targetScale - currentScale) * 0.15f;
//        mandelbrotSprite.setScale(currentScale, currentScale);
//        mandelbrotSprite.setOrigin(width / 2.f, height / 2.f);
//        mandelbrotSprite.setPosition(width / 2.f, height / 2.f);
//
//        window.clear();
//        window.draw(mandelbrotSprite);
//        window.display();
//    }
//
//    // Stoppe le worker proprement
//    running = false;
//    cv.notify_all();
//    worker.join();
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SFML/Graphics.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <atomic>
#include <vector>
#include <cmath>
#include <iostream>

using namespace sf;
using namespace std;

//struct MyView {
//    double xmin, xmax, ymin, ymax;
//
//    MyView zoomAround(double cx, double cy, double zoomFactor) const {
//        double width = (xmax - xmin) * zoomFactor;
//        double height = (ymax - ymin) * zoomFactor;
//        return {
//            cx - width / 2.0,
//            cx + width / 2.0,
//            cy - height / 2.0,
//            cy + height / 2.0
//        };
//    }
//};
//
//class FractalBuilder {
//public:
//    static float computeIterations(double cx, double cy, int maxIter) {
//        double x = 0.0, y = 0.0;
//        int i = 0;
//        while (x * x + y * y <= 4 && i < maxIter) {
//            double xNew = x * x - y * y + cx;
//            y = 2 * x * y + cy;
//            x = xNew;
//            ++i;
//        }
//        return static_cast<float>(i);
//    }
//
//    static Color hsvToRGB(float H, float S, float V) {
//        H = fmodf(H, 360.f);
//        if (H < 0) H += 360.f;
//
//        float s = S / 100.f, v = V / 100.f;
//        float C = s * v;
//        float X = C * (1 - fabsf(fmodf(H / 60.f, 2.f) - 1.f));
//        float m = v - C;
//        float r, g, b;
//
//        if (H < 60) { r = C; g = X; b = 0; }
//        else if (H < 120) { r = X; g = C; b = 0; }
//        else if (H < 180) { r = 0; g = C; b = X; }
//        else if (H < 240) { r = 0; g = X; b = C; }
//        else if (H < 300) { r = X; g = 0; b = C; }
//        else { r = C; g = 0; b = X; }
//
//        return Color(
//            static_cast<Uint8>((r + m) * 255),
//            static_cast<Uint8>((g + m) * 255),
//            static_cast<Uint8>((b + m) * 255)
//        );
//    }
//
//    static void generatePixels(const MyView& v, int iter, unsigned W, unsigned H,
//        std::vector<Uint8>& out)
//    {
//        constexpr int CYCLE = 64;          // longueur d’un cycle de couleur
//        out.resize(W * H * 4);
//
//        for (unsigned y = 0; y < H; ++y)
//            for (unsigned x = 0; x < W; ++x)
//            {
//                double real = v.xmin + (v.xmax - v.xmin) * x / (W - 1.0);
//                double imag = v.ymin + (v.ymax - v.ymin) * y / (H - 1.0);
//
//                // --- itérations « lisses » ------------------------------------
//                double zx = 0.0, zy = 0.0;
//                int i = 0;
//                while (zx * zx + zy * zy <= 4.0 && i < iter)
//                {
//                    double xNew = zx * zx - zy * zy + real;
//                    zy = 2 * zx * zy + imag;
//                    zx = xNew;
//                    ++i;
//                }
//                float mu = i;
//                if (i < iter)                         // point divergent : lissage
//                    mu = i - log2(log2(zx * zx + zy * zy));
//
//                // --- palette arc‑en‑ciel cyclique ------------------------------
//                float t = fmod(mu, CYCLE) / CYCLE;    // [0,1[
//                Color c = hsvToRGB(360.f * t, 100.f, (i < iter ? 100.f : 0.f));
//
//                size_t p = 4ull * (y * W + x);
//                out[p + 0] = c.r;  out[p + 1] = c.g;  out[p + 2] = c.b;  out[p + 3] = 255;
//            }
//    }
//
//};
//
//class RenderWorker {
//public:
//    struct Job { MyView view; int iterations; };
//
//    RenderWorker(unsigned w, unsigned h) : W(w), H(h), thread(&RenderWorker::run, this) {}
//
//    ~RenderWorker() {
//        stop();
//    }
//
//    void stop() {
//        running = false;
//        cv.notify_all();
//        if (thread.joinable())
//            thread.join();
//    }
//
//    void requestJob(const Job& job) {
//        lock_guard lock(mtx);
//        if (!pending) {
//            pending = job;
//            cv.notify_one();
//        }
//    }
//
//    bool getPixels(vector<Uint8>& pixelsOut) {
//        if (ready.exchange(false)) {
//            lock_guard lock(mtx);
//            pixelsOut.swap(latestPixels);
//            return true;
//        }
//        return false;
//    }
//
//private:
//    unsigned W, H;
//    thread thread;
//    mutex mtx;
//    condition_variable cv;
//    optional<Job> pending;
//    vector<Uint8> latestPixels;
//    atomic<bool> ready{ false };
//    atomic<bool> running{ true };
//
//    void run() {
//        vector<Uint8> localBuffer;
//        while (running) {
//            Job job;
//            {
//                unique_lock lock(mtx);
//                cv.wait(lock, [&] { return !running || pending.has_value(); });
//                if (!running) break;
//                job = *pending;
//                pending.reset();
//            }
//            FractalBuilder::generatePixels(job.view, job.iterations, W, H, localBuffer);
//            {
//                lock_guard lock(mtx);
//                latestPixels = std::move(localBuffer);
//                ready = true;
//            }
//        }
//    }
//};
//
//class FractalApp {
//public:
//    FractalApp() :
//        window(VideoMode(W, H), "Mandelbrot OO"),
//        tex(), sprite(), font(), info("", font, 22),
//        worker(W, H),
//        view({ -2.4, 1.0, -1.425, 1.425 }),
//        iter(80), frame(0)
//    {
//        tex.create(W, H);
//        sprite.setTexture(tex);
//        font.loadFromFile("arial.ttf");
//        info.setPosition(15, 15);
//        worker.requestJob({ view, iter });
//    }
//
//    void run() {
//        Clock logic;
//        const Time tick = milliseconds(60);
//        while (window.isOpen()) {
//            handleEvents();
//            updateLogic(logic, tick);
//            render();
//        }
//    }
//
//private:
//    const unsigned W = 1600, H = 900;
//    const double targetR = -0.7453, targetI = 0.1127;
//    const double zoomFactor = 0.99;
//
//    RenderWindow window;
//    Texture tex;
//    Sprite sprite;
//    Font font;
//    Text info;
//
//    RenderWorker worker;
//    MyView view;
//    int iter;
//    unsigned long long frame;
//
//    void handleEvents() {
//        Event e;
//        while (window.pollEvent(e)) {
//            if (e.type == Event::Closed)
//                window.close();
//        }
//    }
//
//    void updateLogic(Clock& clock, const Time& tick) {
//        if (clock.getElapsedTime() > tick) {
//            view = view.zoomAround(targetR, targetI, zoomFactor);
//            iter = min(5000, int(iter * 1.015));
//            worker.requestJob({ view, iter });
//            clock.restart();
//        }
//
//        vector<Uint8> buf;
//        if (worker.getPixels(buf)) {
//            tex.update(buf.data());
//            frame = 0;
//        }
//
//        double visFactor = pow(zoomFactor, frame / (double)(tick.asMilliseconds() / 16));
//        double cw = (view.xmax - view.xmin) / visFactor;
//        double ch = (view.ymax - view.ymin) / visFactor;
//
//        sprite.setScale(
//            static_cast<float>(W) / static_cast<float>(cw),
//            static_cast<float>(H) / static_cast<float>(ch)
//        );
//        sprite.setPosition(0, 0);
//        ++frame;
//    }
//
//    void render() {
//        info.setString("Iterations: " + to_string(iter));
//        window.clear();
//        window.draw(sprite);
//        window.draw(info);
//        window.display();
//    }
//};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//int main()
//{
//    //Fractal();
//    //ExecuteMaze();
//
//    FractalApp app;
//    app.run();
//
//    return 0;
//}

#include <SFML/Graphics.hpp>
#include <complex>
#include <vector>
#include <iostream>
#include <cmath>

class ColorPalette {
    std::vector<sf::Color> colors;
public:
    ColorPalette() {
        colors = {
            sf::Color(66, 30, 15),
            sf::Color(25, 7, 26),
            sf::Color(9, 1, 47),
            sf::Color(4, 4, 73),
            sf::Color(0, 7, 100),
            sf::Color(12, 44, 138),
            sf::Color(24, 82, 177),
            sf::Color(57, 125, 209),
            sf::Color(134, 181, 229),
            sf::Color(211, 236, 248),
            sf::Color(241, 233, 191),
            sf::Color(248, 201, 95),
            sf::Color(255, 170, 0),
            sf::Color(204, 128, 0),
            sf::Color(153, 87, 0),
            sf::Color(106, 52, 3)
        };
    }

    sf::Color getColor(double t) const {
        if (t <= 0) return colors.front();
        if (t >= 1) return colors.back();

        double scaled = t * (colors.size() - 1);
        int i = (int)scaled;
        double frac = scaled - i;

        sf::Color c1 = colors[i];
        sf::Color c2 = colors[i + 1];

        sf::Uint8 r = (sf::Uint8)(c1.r + frac * (c2.r - c1.r));
        sf::Uint8 g = (sf::Uint8)(c1.g + frac * (c2.g - c1.g));
        sf::Uint8 b = (sf::Uint8)(c1.b + frac * (c2.b - c1.b));

        return sf::Color(r, g, b);
    }
};

class Mandelbrot {
public:
    static int computeIterations(double x0, double y0, int maxIter) {
        double x = 0, y = 0;
        int iter = 0;
        while (x * x + y * y <= 4 && iter < maxIter) {
            double xtemp = x * x - y * y + x0;
            y = 2 * x * y + y0;
            x = xtemp;
            iter++;
        }
        return iter;
    }
};

class ViewController {
    double centerX, centerY;
    double zoom; // plus grand = plus zoomé (ex : 200)

public:
    ViewController() : centerX(-0.75), centerY(0.1), zoom(300) {}

    void smoothZoomAuto(double factor, double targetX, double targetY) {
        // Zoom progressif en gardant cible fixe
        // transform coords selon zoom :
        // Avant zoom, coord fractales cible = targetX, targetY
        // On garde ce point fixe
        double newZoom = zoom * factor;

        // Ajuster centre pour que target reste à la même position à l'écran
        centerX = targetX - (targetX - centerX) * (zoom / newZoom);
        centerY = targetY - (targetY - centerY) * (zoom / newZoom);
        zoom = newZoom;
    }

    double getCenterX() const { return centerX; }
    double getCenterY() const { return centerY; }
    double getZoom() const { return zoom; }
};

class RendererSFML {
    sf::Image image;
    sf::Texture texture;
    sf::Sprite sprite;
    const ColorPalette& palette;
    int width, height;

    int maxIter;
    ViewController view;

    void renderBand(int yStart, int yEnd) {
        double cx = view.getCenterX();
        double cy = view.getCenterY();
        double zoom = view.getZoom();

        for (int py = yStart; py < yEnd; ++py) {
            for (int px = 0; px < width; ++px) {
                double x0 = cx + (px - width / 2.0) / zoom;
                double y0 = cy + (py - height / 2.0) / zoom;

                int iter = Mandelbrot::computeIterations(x0, y0, maxIter);
                double t = (double)iter / maxIter;
                sf::Color color = palette.getColor(t);
                image.setPixel(px, py, color);
            }
        }
    }

public:
    RendererSFML(int w, int h, const ColorPalette& pal)
        : width(w), height(h), palette(pal) {
        image.create(width, height, sf::Color::Black);
        texture.create(width, height);
        sprite.setTexture(texture);
    }

    void render(const ViewController& view_, int maxIter_) {
        maxIter = maxIter_;
        view = view_;

        const int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) {
            // fallback si pas détecté
            renderBand(0, height);
            texture.update(image);
            return;
        }

        std::vector<std::thread> threads;
        int bandHeight = height / numThreads;

        for (int i = 0; i < numThreads; ++i) {
            int yStart = i * bandHeight;
            int yEnd = (i == numThreads - 1) ? height : yStart + bandHeight;
            threads.emplace_back(&RendererSFML::renderBand, this, yStart, yEnd);
        }

        for (auto& t : threads)
            t.join();

        texture.update(image);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(sprite);
    }
};

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Seahorse Valley Mandelbrot Zoom");
    window.setFramerateLimit(60);

    Clock clock;
    float fps = 0.f;
    float fpsUpdateTime = 0.f;

    ColorPalette palette;
    RendererSFML renderer(WIDTH, HEIGHT, palette);
    ViewController view;

    int maxIter = 300;
    const double zoomFactor = 1.03; // Zoom progressif (5% par frame)

    // Coordonnée fixe Seahorse Valley
    const double targetX = -0.74543;
    const double targetY = 0.11301;

    Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "'arial.ttf' n'est pas accessible !" << endl;
    }

    Text fpsText;
    fpsText.setFont(font);
    fpsText.setCharacterSize(18);
    fpsText.setFillColor(sf::Color::White);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Add || event.key.code == sf::Keyboard::Equal)
                    maxIter += 50;
                else if (event.key.code == sf::Keyboard::Subtract && maxIter > 50)
                    maxIter -= 50;
            }
        }

        float deltaTime = clock.restart().asSeconds();
        fpsUpdateTime += deltaTime;
        fps = 1.f / deltaTime;

        // Mettre à jour l’affichage FPS une fois par seconde (optionnel)
        if (fpsUpdateTime >= 1.f) {
            fpsUpdateTime = 0.f;
            fpsText.setString("FPS: " + std::to_string((int)fps));
            // Position en haut à droite (10px du bord)
            fpsText.setPosition(WIDTH - fpsText.getLocalBounds().width - 10, 10);
        }


        // Zoom automatique vers Seahorse Valley
        view.smoothZoomAuto(zoomFactor, targetX, targetY);

        renderer.render(view, maxIter);

        window.clear();
        renderer.draw(window);
        window.draw(fpsText);
        window.display();
    }

    return 0;
}
