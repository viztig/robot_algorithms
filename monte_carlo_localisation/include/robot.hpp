#include "utility.hpp"
#pragma once
class Robot
{
public:
    double x, y, theta;
    double world_size = 100;
    double forward_noise, turn_noise, sensor_noise;
    Robot()
    {
        x = util::gen_random() * world_size;
        y = util::gen_random() * world_size;
        theta = util::gen_random() * 6.28;
        forward_noise = 0.0;
        turn_noise = 0.0;
        sensor_noise = 0.0;
    }
    ~Robot() {}
    double get_x() { return x; }
    double get_y() { return y; }
    double get_theta() { return theta; }
    std::vector<double> get_state() const;
    void set_state(double x, double y, double theta);
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    std::vector<double> sense(std::vector<std::vector<double>> &landmark) const;
    void move(double forward, double turn);
    void show() const;
    double measurement_prob(std::vector<double> &measurement, std::vector<std::vector<double>> &landmark) const;
    double error(Robot p[], int sample_size);
};
std::vector<double> Robot::get_state() const
{
    return {this->x, this->y, this->theta};
}
void Robot::set_state(double _x, double _y, double _theta)
{
    if (_x < 0 || _x >= this->world_size)
    {
        throw std::invalid_argument("x coordinate out of bound");
    }
    if (_y < 0 || _y >= this->world_size)
    {
        throw std::invalid_argument("y coordinate out of bound");
    }
    if (_theta < 0 || _theta >= 6.28)
    {
        throw std::invalid_argument("theta coordinate out of bound");
    }
    this->x = _x;
    this->y = _y;
    this->theta = _theta;
    // this->theta = _theta - 3.14 * int(_theta / 3.14);
}
void Robot::set_noise(double _forward_noise, double _turn_noise, double _sensor_noise)
{
    this->forward_noise = _forward_noise;
    this->turn_noise = _turn_noise;
    this->sensor_noise = _sensor_noise;
}
std::vector<double> Robot::sense(std::vector<std::vector<double>> &landmark) const
{
    std::vector<double> sense(landmark.size(), 0);
    double dist = 0;
    for (int i = 0; i < landmark.size(); i++)
    {
        dist = sqrt(pow(this->x - landmark[i][0], 2) + pow(this->y - landmark[i][1], 2));
        dist = dist + util::gen_gaussian_random(0.0, this->sensor_noise);
        sense[i] = dist;
    }
    return sense;
}
void Robot::move(double forward, double turn)
{
    this->theta = this->theta + turn + util::gen_gaussian_random(0.0, this->turn_noise);
    double dist = forward + util::gen_gaussian_random(0.0, this->forward_noise);
    this->theta = mod(this->theta, 6.28);
    this->x = this->x + (cos(this->theta) * dist);
    this->y = this->y + (cos(this->theta) * dist);
    this->x = mod(this->x, world_size);
    this->y = mod(this->y, world_size);
    this->set_state(this->x, this->y, this->theta);
    this->set_noise(this->forward_noise, this->turn_noise, this->sensor_noise);
}
void Robot::show() const
{
    std::cout << "x= " << this->x << "  y= " << this->y << "  theta= " << this->theta << std::endl;
}
double Robot::measurement_prob(std::vector<double> &measurement, std::vector<std::vector<double>> &landmark) const
{
    double p = 1.0;
    double dist = 0;
    for (int i = 0; i < landmark.size(); i++)
    {
        dist = sqrt(pow(this->x - landmark[i][0], 2) + pow(this->y - landmark[i][1], 2));
        p = p * util::get_gaussian_prob(measurement[i], dist, this->sensor_noise);
    }
    return p;
}
double Robot::error(Robot p[], int sample_size)
{
    double er = 0.0;
    for (int i = 0; i < sample_size; i++)
    {
        double dx = mod((p[i].x - this->x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - this->y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double e = sqrt(pow(dx, 2) + pow(dy, 2));
        er += e;
    }
    return er / sample_size;
}
