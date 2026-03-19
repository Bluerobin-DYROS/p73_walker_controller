#ifndef QUADRATICPROGRAM_H
#define QUADRATICPROGRAM_H

#include <Eigen/Dense>
#include "math_type_define.h"
#include "OsqpEigen/OsqpEigen.h"    // Refer to https://robotology.github.io/osqp-eigen/class_osqp_eigen_1_1_settings.html

class CQuadraticProgram
{
public:
    CQuadraticProgram();
    virtual ~CQuadraticProgram();

    void InitializeProblemSize(const int &num_var, const int &num_cons);
    void UpdateMinProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g);
    void UpdateSubjectToAx(const Eigen::MatrixXd &A, const Eigen::VectorXd &lbA, const Eigen::VectorXd &ubA);
    void UpdateSubjectToX(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);

    void DeleteSubjectToAx();
    void DeleteSubjectToX();

    void PrintHessGrad();
    void PrintSubjectToAx();
    void PrintSubjectTox();

    bool solveQP(const unsigned int &num_max_iter, Eigen::VectorXd &QPSolution);
        void setWarmStartOption();


private:
    bool _Initialized;

    int _num_var;
    int _num_cons;

    Eigen::VectorXd _x;

    Eigen::MatrixXd _H;
    Eigen::VectorXd _g;

    bool _bool_constraint_Ax;
    Eigen::MatrixXd _A;
    Eigen::VectorXd _lbA;
    Eigen::VectorXd _ubA;

    bool _bool_constraint_x;
    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;

    void Initialize();

    bool _bool_warm_start = false;
};

#endif // QUADRATICPROGRAM_H
