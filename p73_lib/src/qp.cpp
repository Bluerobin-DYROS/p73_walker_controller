#include "qp.h"

CQuadraticProgram::CQuadraticProgram()
{
    Initialize();
}
CQuadraticProgram::~CQuadraticProgram()
{
}

void CQuadraticProgram::Initialize()
{
    _num_var = 1;
    _num_cons = 1;
}

void CQuadraticProgram::InitializeProblemSize(const int &num_var, const int &num_cons)
{
    _bool_constraint_Ax = false;
    _bool_constraint_x  = false;

    _num_var = num_var;
    _num_cons = num_cons;

    _x.setZero(_num_var);
    
    _H.setZero(_num_var, _num_var);
    _g.setZero(_num_var);

    _A.setZero(_num_cons, _num_var);
    _lbA.setZero(_num_cons);
    _ubA.setZero(_num_cons);

    _lb.setZero(_num_var);
    _ub.setZero(_num_var);

    _Initialized = false; 
}

void CQuadraticProgram::UpdateMinProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g)
{
    _H = H;
    _g = g;
}

void CQuadraticProgram::UpdateSubjectToAx(const Eigen::MatrixXd &A, const Eigen::VectorXd &lbA, const Eigen::VectorXd &ubA)
{
    _A   = A;
    _lbA = lbA;
    _ubA = ubA;
    _bool_constraint_Ax = true;
}

void CQuadraticProgram::UpdateSubjectToX(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub)
{
    _lb = lb;
    _ub = ub;
    _bool_constraint_x = true;
}

void CQuadraticProgram::DeleteSubjectToAx()
{
    _bool_constraint_Ax = false;
    _A.resize(0, _num_var);
    _lbA.resize(0);
    _ubA.resize(0);
}

void CQuadraticProgram::DeleteSubjectToX()
{
    _bool_constraint_x = false;
    _lb.setConstant(_num_var, -std::numeric_limits<double>::infinity());
    _ub.setConstant(_num_var,  std::numeric_limits<double>::infinity());
}

void CQuadraticProgram::PrintHessGrad()
{
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << "----------------------------------    H    -----------------------------------\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << _H << "\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << "----------------------------------    g    -----------------------------------\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << _g.transpose() << "\n";
    std::cout << "------------------------------------------------------------------------------\n";
}

void CQuadraticProgram::PrintSubjectToAx()
{
    if (_bool_constraint_Ax)
    {
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "----------------------------------    A    -----------------------------------\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << _A << "\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "---------------------------------    lbA    ----------------------------------\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << _lbA.transpose() << "\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "---------------------------------    ubA    ----------------------------------\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << _ubA.transpose() << "\n";
        std::cout << "------------------------------------------------------------------------------\n";
    }
    else
    {
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "                   s.t. lbA <= Ax <= ubA is not inserted.                     \n";
        std::cout << "------------------------------------------------------------------------------\n";
    }
}

void CQuadraticProgram::PrintSubjectTox()
{
    if (_bool_constraint_x)
    {
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "---------------------------------    lb    -----------------------------------\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << _lb.transpose() << "\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "---------------------------------    ub    -----------------------------------\n";
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << _ub.transpose() << "\n";
        std::cout << "------------------------------------------------------------------------------\n";
    }
    else
    {
        std::cout << "------------------------------------------------------------------------------\n";
        std::cout << "                     s.t. lb <= x <= ub is not inserted.                      \n";
        std::cout << "------------------------------------------------------------------------------\n";
    }
}

bool CQuadraticProgram::solveQP(const unsigned int &num_max_iter, Eigen::VectorXd &QPSolution)
{
    Eigen::SparseMatrix<double> H(_num_var, _num_var);
    Eigen::VectorXd g;

    Eigen::SparseMatrix<double> A(_num_cons, _num_var);
    Eigen::VectorXd lbA, ubA;

    H = _H.sparseView();
    A = _A.sparseView();

    g = _g;
    lbA = _lbA;
    ubA = _ubA;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(_bool_warm_start);

    // set the maximum iteration of ADMM
    solver.settings()->setMaxIteration(num_max_iter);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(_num_var);
    solver.data()->setNumberOfConstraints(_num_cons);

    if (!solver.data()->setHessianMatrix(H))                return false;
    if (!solver.data()->setGradient(g))                     return false;
    if (!solver.data()->setLinearConstraintsMatrix(A))      return false;
    if (!solver.data()->setLowerBound(lbA))                 return false;
    if (!solver.data()->setUpperBound(ubA))                 return false;

    // instantiate the solver
    if (!solver.initSolver())   return false;

    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)   return false;
    if (solver.getStatus() != OsqpEigen::Status::Solved)   return false;
    
    // get the controller input
    QPSolution = solver.getSolution();

    solver.clearSolverVariables();
    solver.clearSolver();

    return true;
}

void CQuadraticProgram::setWarmStartOption()
{
    _bool_warm_start = true;
}