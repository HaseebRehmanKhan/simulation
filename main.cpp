#include "raylib.h"
#include <algorithm>
#include <vector>
#include <cmath>


bool collidesWithAnyWall(
    Vector2 position,
    float radius,
    const std::vector<Rectangle> &walls
) {
    for (const Rectangle &wall: walls) {
        if (CheckCollisionCircleRec(position, radius, wall)) {
            return true;
        }
    }
    return false;
}


class LidarSensor {
private:
    float angle = 0.0f;
    float range = 200.0f;
    float rotationSpeed = 2.0f;
    float detectedDistance = 200.0f;

public:
    void update(
        float dt,
        float robotX,
        float robotY,
        const std::vector<Rectangle> &walls,
        float sensorStep) {
        angle += rotationSpeed * dt;
        detectedDistance = range;
        bool hit = false;
        for (float distance = 0.0f;
             distance <= range && !hit;
             distance += sensorStep) {
            Vector2 sensorPoint = {
                robotX + distance * std::cos(angle),
                robotY + distance * std::sin(angle )
            };
            for (const Rectangle& wall : walls) {
                if (CheckCollisionPointRec(sensorPoint, wall)) {
                    hit = true;
                    detectedDistance = distance;
                    break;
                }
            }


        };
    }

    float getAngle() const {
        return angle;
    }

    float getDetectedDistance() const {
        return detectedDistance;
    }
    float getRotationSpeed() const {
        return rotationSpeed;
    }
};

int main() {
    const int screenWidth = 1000;
    const int screenHeight = 700;

    float robotX = 200.0f;
    float robotY = 350.0f;


    const float robotSpeed = 250.0f;
    const float robotRadius = 20.0f;
    LidarSensor lidar;
    float sensorStep = 1.0f;

    InitWindow(screenWidth, screenHeight, "Robot Simulator");
    SetTargetFPS(60);

    std::vector<Rectangle> walls = {
        {400.0, 200.0, 100.0, 250.0},
        {650.0f, 100.0f, 80.0f, 300.0f},
        {150.0f, 500.0f, 400.0f, 50.0f}
    };


    while (!WindowShouldClose()) {
        // 1. Read input and update position
        float dt = GetFrameTime();
        float proposedX = robotX;
        float proposedY = robotY;

        if (IsKeyDown(KEY_W)) proposedY -= robotSpeed * dt;
        if (IsKeyDown(KEY_S)) proposedY += robotSpeed * dt;
        if (IsKeyDown(KEY_A)) proposedX -= robotSpeed * dt;
        if (IsKeyDown(KEY_D)) proposedX += robotSpeed * dt;

        proposedX = std::clamp(proposedX, robotRadius, screenWidth - robotRadius);
        proposedY = std::clamp(proposedY, robotRadius, screenHeight - robotRadius);


        Vector2 proposedHorizontal = {proposedX, robotY};

        if (!collidesWithAnyWall(proposedHorizontal, robotRadius, walls)) {
            robotX = proposedX;

        }

        Vector2 proposedVertical = {robotX, proposedY};

        if (!collidesWithAnyWall(proposedVertical, robotRadius, walls)) {
            robotY = proposedY;
        }
        lidar.update(dt, robotX, robotY, walls, sensorStep);






        float rayEndX = robotX + lidar.getDetectedDistance() * std::cos(lidar.getAngle());
        float rayEndY = robotY + lidar.getDetectedDistance() * std::sin(lidar.getAngle());

        // 2. Draw the scene
        BeginDrawing();
        ClearBackground(BLACK);
        DrawLine(
            static_cast<int>(robotX),
            static_cast<int>(robotY),
            static_cast<int>(rayEndX),
            static_cast<int>(rayEndY),
            GREEN
        );
        for (const Rectangle &wall: walls) {
            DrawRectangleRec(wall, RED);
        }


        DrawCircle(
            static_cast<int>(robotX),
            static_cast<int>(robotY),
            robotRadius,
            BLUE
        );

        DrawText("I LOVE ROBOTICS", 20, 20, 24, RAYWHITE);
        DrawText("HIRE ME JIM FAN", 20, 80, 24, RAYWHITE);
        DrawText(
            TextFormat("LiDAR distance: %.1f px", lidar.getDetectedDistance()),
            20, 140, 24, GREEN
        );

        EndDrawing();
    }
    CloseWindow();
}
