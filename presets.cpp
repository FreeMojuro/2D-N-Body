#include "presets.hpp"
#include <deque>
#include <cmath>
#include <raylib.h>


template<typename T>
void inline Preset1(std::deque<T>& bodies){
    bodies.clear();
    bodies.push_back({Vector2{600,450}, Vector2{0,0}, 10000, 100, Vector2{0,0}, YELLOW});
    bodies.push_back({Vector2{600,800}, Vector2{180,0}, 100, 25, Vector2{0,0}, BLUE});

}

template<typename T>
void inline Preset2(std::deque<T>& bodies){
    bodies.clear();
    Color RandColor{static_cast<unsigned char>(GetRandomValue(0, 255)), 
        static_cast<unsigned char>(GetRandomValue(0, 255)), static_cast<unsigned char>(GetRandomValue(0, 255)),255};
    
    for(int x{0}; x < 40; x++ ){
        bodies.emplace_back(Vector2{float(GetRandomValue(0, 1200)),float(GetRandomValue(0, 900))}, Vector2{1,1},50,15,Vector2{0,0},RandColor);
    }

}


template<typename T>
void inline Preset3(std::deque<T>& bodies){
    bodies.clear();
    constexpr double star_mass = 7500.0;
    constexpr double grav_const = 1000.0;
    const Vector2 center{600, 450};
    bodies.push_back({center, Vector2{0,0}, star_mass, 30, Vector2{0,0}, YELLOW}); // star

    const auto add_planet = [&](double orbit_radius, double mass, double size, Color color) {
        const double speed = std::sqrt(grav_const * star_mass / orbit_radius);
        bodies.push_back({Vector2{center.x, center.y + static_cast<float>(orbit_radius)},
                          Vector2{static_cast<float>(speed), 0}, mass, size,
                          Vector2{0,0}, color});
    };
    add_planet(100, 0.6, 5, RED);       // Mercury
    add_planet(165, 1.5, 9, ORANGE);    // Venus
    add_planet(245, 2.0, 10, BLUE);     // Earth
    add_planet(325, 2.0, 10, RED);      //Mars
    add_planet(450, 4.0, 20, BROWN);    //Saturn
    add_planet(550, 6.0, 25, DARKBROWN); //Jupiter
    add_planet(650, 4.0, 15, BLUE);    //Uranus
    add_planet(750, 3.5, 12, DARKBLUE);  //Neptune

    
    double px = 0;
    for (const auto& body : bodies) px += body.mass * body.velocity.x;
    bodies.front().velocity.x = static_cast<float>(-px / bodies.front().mass);


}

template<typename T>
void inline Preset4(std::deque<T>& bodies){
    bodies.clear();
    bodies.push_back({Vector2{600,450}, Vector2{0,0}, 40000, 200, Vector2{0,0}, BLACK});
    
} 


