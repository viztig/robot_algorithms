#include "include/robot.hpp"

int main()
{
    // Landmarks
    std::vector<std::vector<double>> landmarks = {{20.0, 20.0}, {20.0, 80.0}, {20.0, 50.0}, {50.0, 20.0}, {50.0, 80.0}, {80.0, 80.0}, {80.0, 20.0}, {80.0, 50.0}};
    double world_size = 100;
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set_state(30.0, 50.0, 3.14 / 2.0);
    robot.move(15.0, -3.14 / 2.0);
    robot.move(10.0, -3.14 / 2.0);

    int sample_size = 1000;
    std::vector<double> measurement(50, 0);
    Robot particle[sample_size];
    for (int i = 0; i < sample_size; i++)
    {
        particle[i].set_noise(0.05, 0.05, 5);
        // particle[i].set_noise(robot.forward_noise, robot.turn_noise, robot.sensor_noise);
    }
    // robot = Robot();
    for (int i = 0; i < 50; i++)
    {
        // std::cout << i << std::endl;
        robot.move(5, 0.1);
        // robot.show();
        measurement = robot.sense(landmarks);
        double weight[sample_size];
        double total_weight = 0;
        for (int i = 0; i < sample_size; i++)
        {
            weight[i] = particle[i].measurement_prob(measurement, landmarks);
            total_weight = total_weight + weight[i];
        }
        Robot resample_particle[sample_size];
        std::vector<double> normalized_prob(sample_size, 0);
        for (int i = 0; i < sample_size; i++)
        {
            normalized_prob[i] = weight[i] / total_weight;
        }
        double spacing = total_weight / sample_size;
        double start_point = util::gen_random() * spacing;
        int index = 0;
        for (int i = 0; i < sample_size; i++)
        {
            double pointer = start_point + i * spacing;
            while (pointer > normalized_prob[i])
            {
                index = (index + 1) % sample_size;
            }
            resample_particle[i] = particle[index];
        }
        double dx = 0, dy = 0;
        for (int i = 0; i < sample_size; i++)
        {
            particle[i] = resample_particle[i];
            dx += particle[i].x;
            dy += particle[i].y;
        }
        std::cout << robot.error(particle, sample_size) << std::endl;
        // std::cout << robot.get_x() << std::endl;
    }
}
