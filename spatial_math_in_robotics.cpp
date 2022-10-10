// IN RADIANS
//Markus Buchholz

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using Eigen::MatrixXd;
using Eigen::MatrixXf;

Eigen::MatrixXf Rotx(float a)
{

    Eigen::MatrixXf rot(4, 3);
    rot << 1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0;
    return rot;
}

Eigen::MatrixXf Roty(float a)
{

    Eigen::MatrixXf rot(4, 3);
    rot << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a), 0, 0, 0;
    return rot;
}

Eigen::MatrixXf Rotz(float a)
{

    Eigen::MatrixXf rot(4, 3);
    rot << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1, 0, 0, 0;
    return rot;
}

Eigen::MatrixXf Transl(float x, float y, float z)
{

    Eigen::MatrixXf tr(4, 1);
    tr << x, y, z, 1;
    return tr;
}

Eigen::MatrixXf Transfx(float a, float x, float y, float z)
{

    Eigen::MatrixXf H(4, 4);

    H << Rotx(a), Transl(x, y, z);

    return H;
}

Eigen::MatrixXf Transfy(float a, float x, float y, float z)
{

    Eigen::MatrixXf H(4, 4);

    H << Roty(a), Transl(x, y, z);

    return H;
}

Eigen::MatrixXf Transfz(float a, float x, float y, float z)
{

    Eigen::MatrixXf H(4, 4);

    H << Rotz(a), Transl(x, y, z);

    return H;
}

Eigen::MatrixXf robotMovement()
{
    Eigen::MatrixX4f Hab(4, 4);
    Eigen::MatrixX4f Hbc(4, 4);
    Eigen::MatrixX4f Had(4, 4);
    Eigen::MatrixXf Hde(4, 4);
    Eigen::MatrixX4f Hce(4, 4);

    Eigen::MatrixXf Ha(4, 4);
    Eigen::MatrixX4f Hb(4, 4);
    Eigen::MatrixX4f Hc(4, 4);
    Eigen::MatrixX4f Hd(4, 4);
    Eigen::MatrixX4f He(4, 4);

    // Hab * Hbc * Hce = Had * Hde
    // Hab = Had * Hdb
    // Hce = ?

    float a30 = M_PI / 6;
    float a45 = M_PI_4;
    float a60 = M_PI / 3;

    Ha << Transfx(0, 0, 0, 0);                                                    // world
    Hb << Transfx(0, 2.8, 0.5, 0);                                                // robot
    Hc << Transfz(-a45, 1, 1, 2) * Transfy(a45, 0, 0, 0) * Transfx(a45, 0, 0, 0); // end-efector
    Hd << Transfz(a60, 2, 3, 3) * Transfy(a60, 0, 0, 0) * Transfx(-a60, 0, 0, 0); // camera
    He << Transfz(a30, 1, 2, -2) * Transfy(a30, 0, 0, 0) * Transfx(a30, 0, 0, 0); // object

    // Hbc control robot - kinematics
    Hbc = Hb * Hc;
    Hce = (Ha * Hb * Hbc).inverse() * Ha * Hd * Hd * He;

    return Hce;
}

int main()
{
    Eigen::MatrixXf Hce(4, 4);
    Hce = robotMovement();
    std::cout << Hce << "\n";
}
