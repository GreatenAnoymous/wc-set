#include "utils.hpp"

void LACAM::info(const int level, const int verbose) { std::cout << std::endl; }

LACAM::Deadline::Deadline(double _time_limit_ms)
    : t_s(Time::now()), time_limit_ms(_time_limit_ms)
{
}

double LACAM::Deadline::elapsed_ms() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() -
                                                                 t_s)
        .count();
}

double LACAM::Deadline::elapsed_ns() const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() - t_s)
        .count();
}

double LACAM::elapsed_ms(const Deadline *deadline)
{
    if (deadline == nullptr)
        return 0;
    return deadline->elapsed_ms();
}

double LACAM::elapsed_ns(const Deadline *deadline)
{
    if (deadline == nullptr)
        return 0;
    return deadline->elapsed_ns();
}

bool LACAM::is_expired(const Deadline *deadline)
{
    if (deadline == nullptr)
        return false;
    return deadline->elapsed_ms() > deadline->time_limit_ms;
}

float LACAM::get_random_float(float from, float to)
{
    static std::random_device rd;
    static std::mt19937 MT(rd());
    std::uniform_real_distribution<float> r(from, to);
    return r(MT);
}
