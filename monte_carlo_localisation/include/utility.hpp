#pragma once
#include <iostream>
#include <random>
#include <math.h>
#include <cmath>
#include <vector>
void print(std::string s)
{
    std::cout << s << std::endl;
}
class util
{
public:
    double mod(double first_term, double second_term);
    static double gen_random();
    static double gen_gaussian_random(double mean, double variance);
    static double get_gaussian_prob(double x, double mean, double variance);
};
double util::gen_random()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}
double util::gen_gaussian_random(double mean, double var)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    // get rangom number from normal distribution
    std::normal_distribution<double> dist(mean, var);
    return dist(gen);
}
double util::get_gaussian_prob(double x, double mean, double var)
{
    double p = exp(-(pow((mean - x), 2)) / (pow(var, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(var, 2)));
    return p;
}
double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}
