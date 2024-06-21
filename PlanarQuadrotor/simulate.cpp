#include "simulate.h"
#include <matplot/matplot.h>
#include <SDL_audio.h>
#include <vector>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void audio_callback(void* userdata, Uint8* stream, int len) {
    SDL_memset(stream, 0, len);
    float* tones = (float*)userdata;
    static float phase1 = 0.0f;
    static float phase2 = 0.0f;
    static float phase3 = 0.0f;
    for (int i = 0; i < len / 4; ++i) {
        float sample = 0.0f;

        sample += std::sinf(phase1 * 2.0f * M_PI);
        sample += 0.5f * std::sinf(2 * phase1 * 2.0f * M_PI);
        sample += 0.25f * std::sinf(3 * phase1 * 2.0f * M_PI);

        sample += std::sinf(phase2 * 2.0f * M_PI);
        sample += 0.5f * std::sinf(2 * phase2 * 2.0f * M_PI);
        sample += 0.25f * std::sinf(3 * phase2 * 2.0f * M_PI);

        sample += std::sinf(phase3 * 2.0f * M_PI);
        sample += 0.5f * std::sinf(2 * phase3 * 2.0f * M_PI);
        sample += 0.25f * std::sinf(3 * phase3 * 2.0f * M_PI);

        sample /= 3.75f;
        ((float*)stream)[i] = sample;
        phase1 += tones[0];
        phase2 += tones[1];
        phase3 += tones[2];
        if (phase1 >= 1.0f) phase1 -= 1.0f;
        if (phase2 >= 1.0f) phase2 -= 1.0f;
        if (phase3 >= 1.0f) phase3 -= 1.0f;
    }
}

int init_audio(SDL_AudioSpec& wav_spec, float* tones) {
    SDL_zero(wav_spec);
    wav_spec.freq = 44100;
    wav_spec.format = AUDIO_F32;
    wav_spec.channels = 1;
    wav_spec.samples = 4096;
    wav_spec.callback = audio_callback;
    wav_spec.userdata = tones;

    if (SDL_OpenAudio(&wav_spec, NULL) < 0) {
        std::cerr << "SDL_OpenAudio Error: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_PauseAudio(0);
    return 0;
}

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(2, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 1, 1, 10, 1, 1, 10;
    R.row(0) << 0.1, 0;
    R.row(1) << 0, 0.1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void plot_trajectory(const std::vector<float>& x_history, const std::vector<float>& y_history) {
    using namespace matplot;
    plot(x_history, y_history);
    xlabel("x");
    ylabel("y");
    title("Trajektoria lotu drona");
    show();
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    /* Timestep for the simulation */
    const float dt = 0.0005f;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    SDL_AudioSpec wav_spec;
    float tones[3] = { 150.0f / 44100.0f, 175.0f / 44100.0f, 200.0f / 44100.0f };

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0 && init_audio(wav_spec, tones) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    // Transformacja wspolrzednych klikniecia na wspolrzedne swiata quadrotora
                    float scale = 50.0f;
                    float goal_x = (x - SCREEN_WIDTH / 2) / scale;
                    float goal_y = -(y - SCREEN_HEIGHT / 2) / scale;

                    // Ustawienie nowego celu
                    goal_state << goal_x, goal_y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYDOWN)
                {
                    if (e.key.keysym.sym == SDLK_c)
                    {
                        // Ustawienie celu na srodek ekranu
                        goal_state << 0, 0, 0, 0, 0, 0;
                        quadrotor.SetGoal(goal_state);
                    }
                    else if (e.key.keysym.sym == SDLK_p)
                    {
                        // Rysowanie trajektorii w osobnym wątku
                        std::thread(plot_trajectory, x_history, y_history).detach();
                    }
                }
            }

            SDL_Delay((int)(dt * 100));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);
            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            // Aktualizacja historii trajektorii
            state = quadrotor.GetState();
            x_history.push_back(state[0]);
            y_history.push_back(state[1]);
            theta_history.push_back(state[2]);

            // Aktualizacja czestotliwosci dzwieku w zaleznosci od predkosci drona
            float speed = sqrt(state[3] * state[3] + state[4] * state[4]);
            tones[0] = 150.0f / 44100.0f + speed * 50.0f / 44100.0f;
            tones[1] = 160.0f / 44100.0f + speed * 50.0f / 44100.0f;
            tones[2] = 170.0f / 44100.0f + speed * 50.0f / 44100.0f;
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
