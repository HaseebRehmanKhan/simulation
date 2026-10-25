
#include "raylib.h"
#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <vector>

// Check whether the robot collides with any wall.
bool collidesWithAnyWall(
    Vector2 position,
    float radius,
    const std::vector<Rectangle>& walls
) {
    for (const Rectangle& wall : walls) {
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

    std::vector<float> detectedDistances =
        std::vector<float>(36, 200.0f);

public:
    void update(
        float dt,
        float robotX,
        float robotY,
        const std::vector<Rectangle>& walls,
        float sensorStep
    ) {
        angle += rotationSpeed * dt;

        for (int i = 0;
             i < static_cast<int>(detectedDistances.size());
             i++) {

            float rayAngle = angle + i * (2 * PI / 36);

            bool hit = false;
            detectedDistances[i] = range;

            for (float distance = 0.0f;
                 distance <= range && !hit;
                 distance += sensorStep) {

                Vector2 sensorPoint = {
                    robotX + distance * std::cos(rayAngle),
                    robotY + distance * std::sin(rayAngle)
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
    // -------------------------------------------------
    // Configuration
    // -------------------------------------------------

    const int screenWidth = 1000;
    const int screenHeight = 700;
    const int cellSize = 20;

    const int gridRows = screenHeight / cellSize;
    const int gridCols = screenWidth / cellSize;

    const int rayCount = 36;
    const float lidarRange = 200.0f;
    const float sensorStep = 1.0f;

    const float robotSpeed = 250.0f;
    const float robotRadius = 20.0f;

    // -------------------------------------------------
    // Simulation state
    // -------------------------------------------------

    float robotX = 200.0f;
    float robotY = 350.0f;

    LidarSensor lidar;

    std::vector<Vector2> pointCloud;

    // -1 = unknown
    //  0 = free
    //  1 = occupied
    std::vector<std::vector<int>> occupancyGrid(
        gridRows,
        std::vector<int>(gridCols, -1)
    );

    std::vector<Rectangle> walls = {
        {400.0f, 200.0f, 100.0f, 250.0f},
        {650.0f, 100.0f, 80.0f, 300.0f},
        {150.0f, 500.0f, 400.0f, 50.0f}
    };

    bool showWalls = true;

    InitWindow(screenWidth, screenHeight, "Robotics Simulation");
    SetTargetFPS(60);

    // =================================================
    // MAIN SIMULATION LOOP
    // =================================================

    while (!WindowShouldClose()) {

        // =============================================
        // PHASE 1: INPUT AND ROBOT MOVEMENT
        // =============================================

        float dt = GetFrameTime();

        float proposedX = robotX;
        float proposedY = robotY;

        if (IsKeyDown(KEY_W)) proposedY -= robotSpeed * dt;
        if (IsKeyDown(KEY_S)) proposedY += robotSpeed * dt;
        if (IsKeyDown(KEY_A)) proposedX -= robotSpeed * dt;
        if (IsKeyDown(KEY_D)) proposedX += robotSpeed * dt;

        proposedX = std::clamp(
            proposedX,
            robotRadius,
            screenWidth - robotRadius
        );

        proposedY = std::clamp(
            proposedY,
            robotRadius,
            screenHeight - robotRadius
        );

        // Resolve horizontal movement first.
        Vector2 proposedHorizontal = {proposedX, robotY};

        if (!collidesWithAnyWall(
                proposedHorizontal,
                robotRadius,
                walls)) {
            robotX = proposedX;
        }

        // Resolve vertical movement second.
        Vector2 proposedVertical = {robotX, proposedY};

        if (!collidesWithAnyWall(
                proposedVertical,
                robotRadius,
                walls)) {
            robotY = proposedY;
        }

        if (IsKeyPressed(KEY_H)) {
            showWalls = !showWalls;
        }

        // =============================================
        // PHASE 2: UPDATE LIDAR SENSOR
        // =============================================

        lidar.update(
            dt,
            robotX,
            robotY,
            walls,
            sensorStep
        );

        // =============================================
        // PHASE 3: UPDATE MAP AND POINT CLOUD
        // No drawing functions in this section.
        // =============================================

        for (int i = 0; i < rayCount; i++) {

            float rayAngle =
                lidar.getAngle() + i * (2 * PI / rayCount);

            float detectedDistance =
                lidar.getDetectedDistance(i);

            // Mark traversed cells as free.
            for (float distance = 0.0f;
                 distance < detectedDistance;
                 distance += sensorStep) {

                Vector2 sensorPoint = {
                    robotX + distance * std::cos(rayAngle),
                    robotY + distance * std::sin(rayAngle)
                };

                // Check world coordinates before converting.
                if (sensorPoint.x < 0.0f ||
                    sensorPoint.x >= screenWidth ||
                    sensorPoint.y < 0.0f ||
                    sensorPoint.y >= screenHeight) {
                    continue;
                }

                int col = static_cast<int>(
                    sensorPoint.x / cellSize
                );

                int row = static_cast<int>(
                    sensorPoint.y / cellSize
                );

                // Preserve cells already marked occupied.
                if (occupancyGrid[row][col] != 1) {
                    occupancyGrid[row][col] = 0;
                }
            }

            // Calculate the ray endpoint.
            float rayEndX =
                robotX + detectedDistance * std::cos(rayAngle);

            float rayEndY =
                robotY + detectedDistance * std::sin(rayAngle);

            // A distance below maximum range indicates a hit.
            if (detectedDistance < lidarRange) {

                Vector2 newPoint = {rayEndX, rayEndY};

                // Mark the endpoint cell as occupied.
                if (newPoint.x >= 0.0f &&
                    newPoint.x < screenWidth &&
                    newPoint.y >= 0.0f &&
                    newPoint.y < screenHeight) {

                    int col = static_cast<int>(
                        newPoint.x / cellSize
                    );

                    int row = static_cast<int>(
                        newPoint.y / cellSize
                    );

                    occupancyGrid[row][col] = 1;
                }

                // Avoid storing points that are too close
                // to existing observations.
                bool alreadyExists = false;

                for (const Vector2& existingPoint : pointCloud) {

                    float distance =
                        Vector2Distance(existingPoint, newPoint);

                    if (distance < 5.0f) {
                        alreadyExists = true;
                        break;
                    }
                }

                if (!alreadyExists) {
                    pointCloud.push_back(newPoint);
                }
            }
        }

        // =============================================
        // PHASE 4: RENDER THE SCENE
        // No simulation-state modifications here.
        // =============================================

        BeginDrawing();
        ClearBackground(BLACK);

        // ---------------------------------------------
        // Draw occupancy grid
        // ---------------------------------------------

        for (int row = 0; row < gridRows; row++) {
            for (int col = 0; col < gridCols; col++) {

                int screenX = col * cellSize;
                int screenY = row * cellSize;

                if (occupancyGrid[row][col] == 1) {
                    DrawRectangle(
                        screenX,
                        screenY,
                        cellSize,
                        cellSize,
                        PURPLE
                    );
                }
                else if (occupancyGrid[row][col] == 0) {
                    DrawRectangle(
                        screenX,
                        screenY,
                        cellSize,
                        cellSize,
                        DARKGREEN
                    );
                }

                DrawRectangleLines(
                    screenX,
                    screenY,
                    cellSize,
                    cellSize,
                    DARKBLUE
                );
            }
        }

        // ---------------------------------------------
        // Draw ground-truth walls
        // ---------------------------------------------

        if (showWalls) {
            for (const Rectangle& wall : walls) {
                DrawRectangleRec(wall, RED);
            }
        }

        // ---------------------------------------------
        // Draw LiDAR rays and current hit markers
        // ---------------------------------------------

        for (int i = 0; i < rayCount; i++) {

            float rayAngle =
                lidar.getAngle() + i * (2 * PI / rayCount);

            float detectedDistance =
                lidar.getDetectedDistance(i);

            float rayEndX =
                robotX + detectedDistance * std::cos(rayAngle);

            float rayEndY =
                robotY + detectedDistance * std::sin(rayAngle);

            DrawLine(
                static_cast<int>(robotX),
                static_cast<int>(robotY),
                static_cast<int>(rayEndX),
                static_cast<int>(rayEndY),
                GREEN
            );

            if (detectedDistance < lidarRange) {
                DrawCircle(
                    static_cast<int>(rayEndX),
                    static_cast<int>(rayEndY),
                    3.0f,
                    YELLOW
                );
            }
        }

        // ---------------------------------------------
        // Draw accumulated point cloud
        // ---------------------------------------------

        for (const Vector2& storedPoint : pointCloud) {
            DrawCircle(
                static_cast<int>(storedPoint.x),
                static_cast<int>(storedPoint.y),
                3.0f,
                ORANGE
            );
        }

        // ---------------------------------------------
        // Draw robot
        // ---------------------------------------------

        DrawCircle(
            static_cast<int>(robotX),
            static_cast<int>(robotY),
            robotRadius,
            BLUE
        );

        // ---------------------------------------------
        // Draw interface
        // ---------------------------------------------

        DrawText("HK", 20, 20, 24, RAYWHITE);

        DrawText(
            TextFormat(
                "LiDAR distance: %.1f px",
                lidar.getDetectedDistance(0)
            ),
            20,
            140,
            24,
            GREEN
        );

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
