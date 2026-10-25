#include <raylib.h>
#include <deque>
#include <cmath>
#include <vector>
#include "presets.cpp"



constexpr int window_width{1200};
constexpr int window_height{900};

constexpr double G = 1000;
constexpr double epsilon = 1;
constexpr float gridSpacing = 50.0f;
constexpr float segmentLength = 10.0f;
double dt;
const int max_trail_size{200};
bool is_paused{false};
bool black_hole_mode{false};
float general_speed_multiplier{1};
int current_preset = 1;


Vector2 operator+(const Vector2& v1, const Vector2& v2){
    return Vector2{v1.x+v2.x, v1.y+v2.y};
}

Vector2 operator+(const Vector2& v1, double x){
    return Vector2{static_cast<float>(v1.x+x), static_cast<float>(v1.y+x)};
}

Vector2 operator-(const Vector2& v1, const Vector2& v2){
    return Vector2{v1.x-v2.x, v1.y-v2.y};
}
Vector2 operator*(const Vector2& v1, const Vector2& v2){
    return Vector2{v1.x*v2.x, v1.y*v2.y};
}

Vector2 operator/(const Vector2& v1, float scalar) {
    return Vector2{v1.x / scalar, v1.y / scalar};
}
Vector2 operator*(const Vector2& v1, double scalar) {
    return Vector2{static_cast<float>(v1.x * scalar), static_cast<float>(v1.y * scalar)};
}

Vector2 operator*(double scalar, const Vector2& v1) {
    return v1 * scalar;
}


class Body{
    public:
    Vector2 pos;
    Vector2 velocity;
    double mass;
    double radius;
    Vector2 a_t;
    Color color;
    std::deque<Vector2> trail;

    Body(Vector2 p, Vector2 v, double m, double r, Vector2 ac, Color col): pos{p}, velocity{v}, mass{m}, radius{r}, a_t{ac}, color{col}{}

    void Draw()const{
        DrawCircleV(pos, radius, color);
    }

    void IntegratePosition(double dt) {
        pos.x += velocity.x * dt + 0.5 * a_t.x * dt * dt;
        pos.y += velocity.y * dt + 0.5 * a_t.y * dt * dt;

        velocity.x += 0.5 * a_t.x * dt;
        velocity.y += 0.5 * a_t.y * dt;
    }

    
    void IntegrateVelocity(Vector2 new_acc, double dt) {
        velocity.x += 0.5 * new_acc.x * dt;
        velocity.y += 0.5 * new_acc.y * dt;
        
        a_t = new_acc; 
    }

    void DrawTrail()const{
        for(int i{0}; i < trail.size() -1 ; ++i){
            unsigned char alpha = static_cast<unsigned char>(255.0f * (static_cast<float>(i) / trail.size()));
            Color fadedcolor = color;
            fadedcolor.a = alpha;
            DrawLineEx(trail[i], trail[i+1],0.5, fadedcolor);
        }
    }

    void UpdTrail(){
        if(trail.size() >= max_trail_size){
            trail.pop_front();
        }
        trail.push_back(pos);

    }

    bool isBlackHole(){
    if(mass  >= 20000)return true;
    else return false;
}
};

Vector2 CalculateForce(const Body& b1, const Body& b2){
    double rx = b1.pos.x - b2.pos.x;
    double ry = b1.pos.y - b2.pos.y;

    double denom = std::pow((rx * rx + ry * ry) + epsilon * epsilon,1.5);
    if(denom == 0)return Vector2{0,0};
    double common = -G * b1.mass * b2.mass;
    
    double Fx = (common/denom) * rx;
    double Fy = (common/denom) * ry;
    return Vector2{static_cast<float>(Fx),static_cast<float>(Fy)};
}

Vector2 Calculate_Accel(const Body& b1, const Body& b2){
    Vector2 force{CalculateForce(b1,b2)};

    float ax = force.x / b1.mass;
    float ay = force.y / b1.mass;
    return Vector2{ax,ay};
}

void AddBody(std::deque<Body>& bodies,Camera2D camera){

    Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
    if(!black_hole_mode){
    Body& sun = bodies[0];
    

    double rx = mouseWorldPos.x - sun.pos.x;
    double ry = mouseWorldPos.y - sun.pos.y;
    double r = std::sqrt(rx * rx + ry * ry);

    double orbital_speed = std::sqrt((G * sun.mass) / r);
    double dir_x = -ry / r;
    double dir_y = rx / r;
    Vector2 initial_velocity = {static_cast<float>(dir_x * orbital_speed),static_cast<float>(dir_y * orbital_speed)};

    Color randColor = {
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        255
    };


    Body b{mouseWorldPos,initial_velocity,25,25,Vector2{0,0},randColor};
    bodies.push_back(b);}


    else{
        Body b{mouseWorldPos,Vector2{0,0},25000,150,Vector2{0,0},BLACK};
        bodies.push_back(b);
    }

}

void DeleteBody(std::deque<Body>& bodies){
    if(bodies.size() > 0)
    bodies.pop_back();


}


void GoFaster(){
    general_speed_multiplier += 0.2;


}

void GoSlower(){
    general_speed_multiplier -= 0.2;

}

Vector2 WarpPoint(Vector2 p, const std::deque<Body>& bodies){
    Vector2 displacement{0,0};
    for(const Body& body: bodies){
        float dx = p.x - body.pos.x;
        float dy = p.y - body.pos.y;
        float r2 = dx * dx + dy * dy + 400.0f;
        float strength = 0.8 * static_cast<float>(body.mass) / r2;

        displacement.x += dx * strength;
        displacement.y += dy * strength;
    }
    return p + displacement;
}

int main(){
    InitWindow(window_width, window_height, "N-Body Simulation - Verlet Integration");
    SetTargetFPS(60);

    Camera2D camera = { 0 };
    camera.target = (Vector2){ window_width/2.0f, window_height/2.0f };
    camera.offset = (Vector2){ window_width / 2.0f, window_height/ 2.0f }; 
    camera.rotation = 0.0f;
    camera.zoom = 1.0f; 

    std::deque<Body> bodies{};
    Preset1(bodies);
   

    while(!WindowShouldClose()){

        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);

            camera.offset = GetMousePosition();
            camera.target = mouseWorldPos;
            camera.zoom += wheel * 0.125f;

            if (camera.zoom < 0.125f) camera.zoom = 0.125f;
            if (camera.zoom > 3.0f) camera.zoom = 3.0f;
        }

        if (IsKeyDown(KEY_RIGHT)) camera.target.x += 16.0f;
        if (IsKeyDown(KEY_LEFT)) camera.target.x -= 16.0f;
        if (IsKeyDown(KEY_DOWN)) camera.target.y += 16.0f;
        if (IsKeyDown(KEY_UP)) camera.target.y -= 16.0f;
        if(IsKeyPressed(KEY_SPACE))is_paused=!is_paused;
        if(IsKeyDown(KEY_E))GoFaster();
        if(IsKeyDown(KEY_Q))GoSlower();
        if(IsKeyPressed(KEY_B))black_hole_mode = !black_hole_mode;
        if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT))AddBody(bodies,camera);
        if(IsKeyPressed(KEY_DELETE))DeleteBody(bodies);
        if(IsKeyDown(KEY_DELETE) && IsKeyDown(KEY_LEFT_SHIFT))DeleteBody(bodies);
        if (IsKeyPressed(KEY_R)) {
            general_speed_multiplier = 1;
            switch(current_preset){
                case 1: Preset1(bodies);break;
                case 2: Preset2(bodies);break;
                case 3: Preset3(bodies);break;
                case 4: Preset4(bodies);break;
            }
            
        }
        if (IsKeyPressed(KEY_ONE) && current_preset != 1){Preset1(bodies); current_preset = 1;}
        if(IsKeyPressed(KEY_TWO) && current_preset != 2){Preset2(bodies); current_preset = 2;}
        if(IsKeyPressed(KEY_THREE) && current_preset != 3){Preset3(bodies); current_preset = 3;}
        if(IsKeyPressed(KEY_FOUR) && current_preset != 4){Preset4(bodies); current_preset = 4;}


        
        dt = GetFrameTime() * general_speed_multiplier;

        for(auto& body : bodies){
            if(!is_paused){
            body.IntegratePosition(dt);
            body.UpdTrail();
        }
        }

        std::vector<Vector2> new_accelerations(bodies.size());
        for(size_t i = 0; i < bodies.size(); ++i){
            Vector2 total_acc{0, 0};
            for(size_t j = 0; j < bodies.size(); ++j){
                if(i == j) continue;
                total_acc = total_acc + Calculate_Accel(bodies[i], bodies[j]);
            }
            new_accelerations[i] = total_acc;
        }

        
        for(size_t i = 0; i < bodies.size(); ++i){
            if(!is_paused) bodies[i].IntegrateVelocity(new_accelerations[i], dt);
        }

        
        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera);

        Vector2 topLeft = GetScreenToWorld2D({0, 0}, camera);
        Vector2 bottomRight = GetScreenToWorld2D(
            {(float)GetScreenWidth(), (float)GetScreenHeight()}, camera);

        float startX = std::floor(topLeft.x / gridSpacing) * gridSpacing;
        float startY = std::floor(topLeft.y / gridSpacing) * gridSpacing;

        
        for (float x = startX; x <= bottomRight.x; x += gridSpacing) {
          for (float y = topLeft.y; y < bottomRight.y; y += segmentLength) {
            DrawLineEx(WarpPoint({x, y}, bodies),
                       WarpPoint({x, y + segmentLength}, bodies), 1, DARKGRAY);
          }
        }

        
        for (float y = startY; y <= bottomRight.y; y += gridSpacing) {
          for (float x = topLeft.x; x < bottomRight.x; x += segmentLength) {
            DrawLineEx(WarpPoint({x, y}, bodies),
                       WarpPoint({x + segmentLength, y}, bodies), 1, DARKGRAY);
          }
        }

        for(const auto& body: bodies){
            body.Draw();
            body.DrawTrail();
        }
        EndMode2D();

        DrawText(TextFormat("FPS:%d", GetFPS()), 10, 10, 20, RAYWHITE);
        DrawText(TextFormat("Num Bodies:%d",bodies.size()),10,40,20,RAYWHITE);
        DrawText(TextFormat("Speed:x%1f",general_speed_multiplier), 10, 70, 20, RAYWHITE);
        if(black_hole_mode)DrawText("Black Hole Mode", 10, 100, 30, RED);
        if(is_paused){
            DrawText("Paused", window_width/2 -170, window_height/2 - 50, 100, RAYWHITE);
            DrawText("DEL: Delete Body", 10, window_height - 30, 20, RAYWHITE);
            DrawText("LEFT CLICK: Add body", 10, window_height - 60, 20, RAYWHITE);
            DrawText("R: Reset", 10, window_height - 90, 20, RAYWHITE);
            DrawText("Q/E: Go Slower/Faster", 10, window_height - 120, 20, RAYWHITE);

            switch(current_preset){
                case 1: DrawText("Preset: Sun-Earth",10,window_height-180,20,RAYWHITE);break;
                case 2: DrawText("Preset: Star Cluster",10,window_height-180,20,RAYWHITE);break;
                case 3: DrawText("Preset: Solar System",10,window_height-180,20,RAYWHITE);break;
                case 4: DrawText("Preset: Single Black Hole",10,window_height-180,20,RAYWHITE);break;
            }
            
        }

        EndDrawing();
    
    }
    CloseWindow();
    return 0;

}
