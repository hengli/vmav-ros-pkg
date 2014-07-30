#include <cstdio>

#include "sensor_models/StereoSensorModel.h"

int main(int argc, char** argv)
{
    Eigen::Vector3d pObstacle(0.0, 0.0, 2.5);

    FILE* pipe = popen("gnuplot -persistent", "w");
    fprintf(pipe, "set xrange [0.000:%f]\n", 25.0);
    fprintf(pipe, "set yrange [0.000:1.000]\n");
    fprintf(pipe, "set title 'Stereo Range Likelihood Model'\n");
    fprintf(pipe, "set xlabel 'Range (m)'\n");
    fprintf(pipe, "set ylabel 'Probability'\n");

    fprintf(pipe, "plot '-' title '2.5 m' with lines, ");
    fprintf(pipe, "'-' title '5.0 m' with lines, ");
    fprintf(pipe, "'-' title '10.0 m' with lines, ");
    fprintf(pipe, "'-' title '20.0 m' with lines\n");

    px::StereoSensorModel sensorModel(0.5, 0.12, 650.0, 0.5, 0.3, 20.0);
    sensorModel.setObstacle(pObstacle);

    for (double r = 0.0; r < 40.0; r += 0.01)
    {
        fprintf(pipe, "%f %f\n", r, sensorModel.getLikelihood(r));
    }

    fprintf(pipe, "e\n");

    pObstacle(2) = 5.0;
    sensorModel.setObstacle(pObstacle);

    for (double r = 0.0; r < 40.0; r += 0.01)
    {
        fprintf(pipe, "%f %f\n", r, sensorModel.getLikelihood(r));
    }

    fprintf(pipe, "e\n");

    pObstacle(2) = 10.0;
    sensorModel.setObstacle(pObstacle);

    for (double r = 0.0; r < 40.0; r += 0.01)
    {
        fprintf(pipe, "%f %f\n", r, sensorModel.getLikelihood(r));
    }

    fprintf(pipe, "e\n");

    pObstacle(2) = 20.0;
    sensorModel.setObstacle(pObstacle);

    for (double r = 0.0; r < 40.0; r += 0.01)
    {
        fprintf(pipe, "%f %f\n", r, sensorModel.getLikelihood(r));
    }

    fprintf(pipe, "e\n");

    pclose(pipe);
    return 0;
}
