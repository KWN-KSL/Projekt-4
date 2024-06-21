#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void drawRect(SDL_Renderer* renderer, int x, int y, int w, int h, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_Rect rect = { x, y, w, h };
    SDL_RenderFillRect(renderer, &rect);
}

void drawEllipse(SDL_Renderer* renderer, int cx, int cy, int rx, int ry, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int w = -rx; w <= rx; w++) {
        for (int h = -ry; h <= ry; h++) {
            if ((w * w * ry * ry + h * h * rx * rx) <= (rx * rx * ry * ry)) {
                SDL_RenderDrawPoint(renderer, cx + w, cy + h);
            }
        }
    }
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    // Przekształcenie współrzędnych do ramki obrazu
    int SCREEN_WIDTH = 1280;
    int SCREEN_HEIGHT = 720;
    float scale = 50.0; // Skalowanie współrzędnych do pikseli
    int img_x = static_cast<int>(q_x * scale + SCREEN_WIDTH / 2);
    int img_y = static_cast<int>(-q_y * scale + SCREEN_HEIGHT / 2); // Ujemne, ponieważ SDL ma (0,0) w lewym górnym rogu

    // Rysowanie ramion drona
    float arm_length = 100.0;
    float arm_width = 9.0;
    float x_offset = arm_length * cos(q_theta) / 2;
    float y_offset = arm_length * sin(q_theta) / 2;
    SDL_Color arm_color = { 0x80, 0x80, 0x80, 0xFF }; // Szary kolor ramion

    // Współrzędne wierzchołków dla ramion
    float cos_theta = cos(q_theta);
    float sin_theta = sin(q_theta);
    SDL_Point arm1[4] = {
        { img_x - static_cast<int>(x_offset + arm_width / 2 * sin_theta), img_y + static_cast<int>(y_offset - arm_width / 2 * cos_theta) },
        { img_x - static_cast<int>(x_offset - arm_width / 2 * sin_theta), img_y + static_cast<int>(y_offset + arm_width / 2 * cos_theta) },
        { img_x + static_cast<int>(x_offset - arm_width / 2 * sin_theta), img_y - static_cast<int>(y_offset + arm_width / 2 * cos_theta) },
        { img_x + static_cast<int>(x_offset + arm_width / 2 * sin_theta), img_y - static_cast<int>(y_offset - arm_width / 2 * cos_theta) }
    };
    SDL_Point arm2[4] = {
        { img_x - static_cast<int>(x_offset - arm_width / 2 * sin_theta), img_y + static_cast<int>(y_offset + arm_width / 2 * cos_theta) },
        { img_x - static_cast<int>(x_offset + arm_width / 2 * sin_theta), img_y + static_cast<int>(y_offset - arm_width / 2 * cos_theta) },
        { img_x + static_cast<int>(x_offset + arm_width / 2 * sin_theta), img_y - static_cast<int>(y_offset - arm_width / 2 * cos_theta) },
        { img_x + static_cast<int>(x_offset - arm_width / 2 * sin_theta), img_y - static_cast<int>(y_offset + arm_width / 2 * cos_theta) }
    };

    // Rysowanie ramion
    SDL_SetRenderDrawColor(gRenderer.get(), arm_color.r, arm_color.g, arm_color.b, arm_color.a);
    SDL_RenderDrawLines(gRenderer.get(), arm1, 4);
    SDL_RenderDrawLines(gRenderer.get(), arm2, 4);

    // Rysowanie korpusu drona
    SDL_Color body_color = { 0x00, 0x00, 0x00, 0xFF };
    int body_rx = static_cast<int>(arm_width * 2); // Promień elipsy w osi x
    int body_ry = static_cast<int>(arm_width * 1.25); // Promień elipsy w osi y
    drawEllipse(gRenderer.get(), img_x, img_y, body_rx, body_ry, body_color);

    // Rysowanie mocowania wirnikow (szarej linii)
    int line_length = 18;
    int line_width = 4;
    SDL_Color line_color = { 0x80, 0x80, 0x80, 0xFF };
    drawRect(gRenderer.get(), img_x - static_cast<int>(x_offset) - line_width / 2, img_y + static_cast<int>(y_offset) - line_length, line_width, line_length, line_color);
    drawRect(gRenderer.get(), img_x + static_cast<int>(x_offset) - line_width / 2, img_y - static_cast<int>(y_offset) - line_length, line_width, line_length, line_color);

    // Rysowanie wirników z efektem obrotu
    SDL_Color rotor_color = { 0x00, 0x00, 0xFF, 0xFF };
    int rotor_radius = 5;
    int time = SDL_GetTicks();
    float angle = (time % 3600) / 25.0f; // Zmiana kątu w czasie
    int rotor1_x = static_cast<int>(rotor_radius * cos(angle));
    int rotor1_y = static_cast<int>(rotor_radius * sin(angle));
    int rotor2_x = static_cast<int>(rotor_radius * cos(angle + M_PI / 2));
    int rotor2_y = static_cast<int>(rotor_radius * sin(angle + M_PI / 2));
    drawEllipse(gRenderer.get(), img_x - static_cast<int>(x_offset) + rotor1_x, img_y + static_cast<int>(y_offset) + rotor1_y - line_length - rotor_radius, rotor_radius, rotor_radius, rotor_color);
    drawEllipse(gRenderer.get(), img_x - static_cast<int>(x_offset) - rotor1_x, img_y + static_cast<int>(y_offset) - rotor1_y - line_length - rotor_radius, rotor_radius, rotor_radius, rotor_color);
    drawEllipse(gRenderer.get(), img_x + static_cast<int>(x_offset) + rotor2_x, img_y - static_cast<int>(y_offset) + rotor2_y - line_length - rotor_radius, rotor_radius, rotor_radius, rotor_color);
    drawEllipse(gRenderer.get(), img_x + static_cast<int>(x_offset) - rotor2_x, img_y - static_cast<int>(y_offset) - rotor2_y - line_length - rotor_radius, rotor_radius, rotor_radius, rotor_color);
}
