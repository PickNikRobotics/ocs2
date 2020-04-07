//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/QpSolver.h"
#include "ocs2_qp_solver/test/testProblemsGeneration.h"

class QpSolverTest : public testing::Test {
 protected:
  static constexpr size_t N_ = 5;
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 2;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_) + nx_;
  static constexpr size_t numConstraints = (N_ + 1) * (nx_ + nc_);

  QpSolverTest() {
    srand(0);
    lqProblem = ocs2::qp_solver::generateRandomLqProblem(N_, nx_, nu_, nc_);
    x0 = ocs2::dynamic_vector_t::Random(nx_);

    cost = ocs2::qp_solver::getCostMatrices(lqProblem, numDecisionVariables);
    constraints = ocs2::qp_solver::getConstraintMatrices(lqProblem, x0, numConstraints, numDecisionVariables);
    std::tie(primalSolution, dualSolution) = ocs2::qp_solver::solveDenseQp(cost, constraints);
  }

  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqProblem;
  ocs2::dynamic_vector_t x0;
  ocs2::qp_solver::ScalarFunctionQuadraticApproximation cost;
  ocs2::qp_solver::VectorFunctionLinearApproximation constraints;
  ocs2::dynamic_vector_t primalSolution;
  ocs2::dynamic_vector_t dualSolution;
};

constexpr size_t QpSolverTest::N_;
constexpr size_t QpSolverTest::nx_;
constexpr size_t QpSolverTest::nu_;
constexpr size_t QpSolverTest::nc_;
constexpr size_t QpSolverTest::numDecisionVariables;
constexpr size_t QpSolverTest::numConstraints;

TEST_F(QpSolverTest, constraintSatisfaction) {
  ASSERT_TRUE(constraints.f.isApprox(-constraints.dfdx * primalSolution));
}

TEST_F(QpSolverTest, firstOrderOptimality) {
  ASSERT_TRUE(cost.dfdx.isApprox(-cost.dfdxx * primalSolution - constraints.dfdx.transpose() * dualSolution));
}

TEST_F(QpSolverTest, costMatrixFullRank) {
  ASSERT_TRUE(cost.dfdxx.fullPivLu().rank() == cost.dfdxx.rows());
}

TEST_F(QpSolverTest, constraintMatrixFullRank) {
  ASSERT_TRUE(constraints.dfdx.fullPivLu().rank() == constraints.dfdx.rows());
}
