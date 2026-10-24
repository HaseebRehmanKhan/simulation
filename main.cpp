#include "raylib.h"
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

#include "raymath.h"


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
    std::vector<float> detectedDistances = std::vector<float>(36, 200);

public:
    void update(
        float dt,
        float robotX,
        float robotY,
        const std::vector<Rectangle> &walls,
        float sensorStep) {
        angle += rotationSpeed * dt;


        for (int i =0; i<detectedDistances.size(); i++) {

            float rayAngle = angle + i * (2 * PI / 36);
            bool hit = false;
            detectedDistances[i] = range;
            for (float distance = 0.0f;
             distance <= range && !hit;
             distance += sensorStep) {
                Vector2 sensorPoint = {
                    robotX + distance * std::cos(rayAngle),
                    robotY + distance * std::sin(rayAngle )
                };
                for (const Rectangle& wall : walls) {
                    if (CheckCollisionPointRec(sensorPoint, wall)) {
                        hit = true;
                        detectedDistances[i] = distance;
                        break;
                    }
                }
             }

        }
    }



    float getAngle() const {
        return angle;
    }

    float getDetectedDistance(int index) const {
        return detectedDistances[index];

    }
    float getRotationSpeed() const {
        return rotationSpeed;
    }



};



int main() {
    const int cellSize = 20;
    const int screenWidth = 1000;
    const int screenHeight = 700;

    float robotX = 200.0f;
    float robotY = 350.0f;


    const float robotSpeed = 250.0f;
    const float robotRadius = 20.0f;
    LidarSensor lidar;
    std::vector<Vector2> pointCloud;
    std::vector<std::vector<int>> occupancyGrid(35, std::vector<int>(50, -1));
    float sensorStep = 1.0f;

    InitWindow(screenWidth, screenHeight, "Simulation");
    SetTargetFPS(60);
    //vector containing walls
    std::vector<Rectangle> walls = {
        {400.0, 200.0, 100.0, 250.0},
        {650.0f, 100.0f, 80.0f, 300.0f},
        {150.0f, 500.0f, 400.0f, 50.0f}
    };

    bool showWalls = true;

    while (!WindowShouldClose()) {
        // 1. Read input and update position
        float dt = GetFrameTime();
        float proposedX = robotX;
        float proposedY = robotY;
        //movement
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


        //toggle walls:
        if (IsKeyPressed(KEY_H)) {
            showWalls = !showWalls;
        }






        //Draw the scene
        BeginDrawing();
        ClearBackground(BLACK);
        //drawing occupancy grid, -1 = unknown, 0 = free, 1 = occupied
        for (int row = 0; row < 35; row++) {
            for (int col = 0; col < 50; col++) {
                int screenX = col * cellSize;
                int screenY = row * cellSize;
                if (occupancyGrid[row][col] == 1) {
                    DrawRectangle(screenX, screenY, cellSize, cellSize, PURPLE);
                }
                DrawRectangleLines(screenX, screenY, cellSize, cellSize, DARKBLUE);
            }
        }
        //for loop for all 36 rays
        //drawing the walls
        if (showWalls) {
            for (const Rectangle &wall: walls) {
                DrawRectangleRec(wall, RED);
            }
        }
        for (int i=0; i<36; i++) {
            //ray angle calculation
            float rayAngle = lidar.getAngle() + i * (2 * PI / 36);
            for (float distance = 0.0f; distance < lidar.getDetectedDistance(i); distance += sensorStep) {
                Vector2 sensorPoint = {
                    robotX + distance * std::cos(rayAngle),
                    robotY + distance * std::sin(rayAngle)

                };
                int row = static_cast<int>(sensorPoint.y / cellSize);
                int col = static_cast<int>(sensorPoint.x / cellSize);
                if (row >= 0 && row <35 && col >= 0 && col < 50) {
                    if (occupancyGrid[row][col] != 1) {
                        occupancyGrid[row][col] = 0;

                    }
                }

            }


            //calculating euclidean ray end points

            float rayEndX = robotX + lidar.getDetectedDistance(i) * std::cos(rayAngle);
            float rayEndY = robotY + lidar.getDetectedDistance(i) * std::sin(rayAngle);

            //drawing the rays

            DrawLine(
            static_cast<int>(robotX),
            static_cast<int>(robotY),
            static_cast<int>(rayEndX),
            static_cast<int>(rayEndY),
               GREEN
            );

            if (lidar.getDetectedDistance(i) < 200) {
                DrawCircle(static_cast<int>(rayEndX), static_cast<int>(rayEndY), static_cast<int>(3.0f), YELLOW);
                bool alreadyExists = false;
                Vector2 newPoint = {rayEndX, rayEndY};
                int column = static_cast<int>(newPoint.x/cellSize);
                int row = static_cast<int>(newPoint.y/cellSize);
                if (column >= 0 && column < 50 && row >= 0 && row < 35) {
                    occupancyGrid[row][column] = 1;
                }

                for (const auto& existingPoint : pointCloud) {

                    float distance = Vector2Distance(existingPoint, newPoint);
                    if (distance < 5) {
                        alreadyExists = true;
                        break;
                    }

                    }
                if (!alreadyExists) {
                    pointCloud.push_back(newPoint);
                }

            }
        }

        for (const auto& storedPoint : pointCloud) {
            DrawCircle(static_cast<int>(storedPoint.x), static_cast<int>(storedPoint.y), static_cast<int>(3.0f), ORANGE);
        }
        //drawing the robot
        DrawCircle(
            static_cast<int>(robotX),
            static_cast<int>(robotY),
            robotRadius,
            BLUE
        );


        //title text
        DrawText("HK", 20, 20, 24, RAYWHITE);
        DrawText(
            TextFormat("LiDAR distance: %.1f px", lidar.getDetectedDistance(0)),
            20, 140, 24, GREEN
        );

        EndDrawing();
    }
    CloseWindow();

    std::vector<std::vector<int>> numbers = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };

}
