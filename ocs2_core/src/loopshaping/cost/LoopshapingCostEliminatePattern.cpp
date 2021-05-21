/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostEliminatePattern.h>

namespace ocs2 {

ScalarFunctionQuadraticApproximation LoopshapingCostEliminatePattern::getQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const CostDesiredTrajectories& desiredTrajectory,
    const PreComputation& preComp) const {
  assert(dynamic_cast<const LoopshapingPreComputation*>(&preComp) != nullptr);

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = *reinterpret_cast<const LoopshapingPreComputation*>(&preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();

  const auto L_system =
      systemCost_->getQuadraticApproximation(t, x_system, u_system, desiredTrajectory, preCompLS.getSystemPreComputation());

  const auto L_filter =
      systemCost_->getQuadraticApproximation(t, x_system, u_filter, desiredTrajectory, preCompLS.getFilteredSystemPreComputation());

  ScalarFunctionQuadraticApproximation L(x.rows(), u.rows());
  L.f = gamma * L_filter.f + (1.0 - gamma) * L_system.f;

  // dfdx
  L.dfdx.head(x_system.rows()).noalias() = gamma * L_filter.dfdx + (1.0 - gamma) * L_system.dfdx;
  if (isDiagonal) {
    L.dfdx.tail(x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getCdiag() * L_system.dfdu;
  } else {
    L.dfdx.tail(x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getC().transpose() * L_system.dfdu;
  }

  // dfdxx
  L.dfdxx.topLeftCorner(x_system.rows(), x_system.rows()).noalias() = gamma * L_filter.dfdxx + (1.0 - gamma) * L_system.dfdxx;
  matrix_t dfduu_C;  // temporary variable, reused in other equation.
  if (isDiagonal) {
    L.dfdxx.topRightCorner(x_system.rows(), x_filter.rows()).noalias() = (1.0 - gamma) * L_system.dfdux.transpose() * s_filter.getCdiag();
    dfduu_C.noalias() = L_system.dfduu * s_filter.getCdiag();
    L.dfdxx.bottomRightCorner(x_filter.rows(), x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getCdiag() * dfduu_C;
  } else {
    L.dfdxx.topRightCorner(x_system.rows(), x_filter.rows()).noalias() = (1.0 - gamma) * L_system.dfdux.transpose() * s_filter.getC();
    dfduu_C.noalias() = L_system.dfduu * s_filter.getC();
    L.dfdxx.bottomRightCorner(x_filter.rows(), x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getC().transpose() * dfduu_C;
  }
  L.dfdxx.bottomLeftCorner(x_filter.rows(), x_system.rows()) = L.dfdxx.topRightCorner(x_system.rows(), x_filter.rows()).transpose();

  // dfdu & dfduu
  L.dfdu = gamma * L_filter.dfdu;
  L.dfduu = gamma * L_filter.dfduu;
  if (isDiagonal) {
    L.dfdu.noalias() += (1.0 - gamma) * s_filter.getDdiag() * L_system.dfdu;
    L.dfduu.noalias() += (1.0 - gamma) * s_filter.getDdiag() * L_system.dfduu * s_filter.getDdiag();
  } else {
    L.dfdu.noalias() += (1.0 - gamma) * s_filter.getD().transpose() * L_system.dfdu;
    L.dfduu.noalias() += (1.0 - gamma) * s_filter.getD().transpose() * L_system.dfduu * s_filter.getD();
  }

  // dfdux
  L.dfdux.leftCols(x_system.rows()) = gamma * L_filter.dfdux;
  if (isDiagonal) {
    L.dfdux.leftCols(x_system.rows()).noalias() += (1.0 - gamma) * s_filter.getDdiag() * L_system.dfdux;
    L.dfdux.rightCols(x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getDdiag() * dfduu_C;
  } else {
    L.dfdux.leftCols(x_system.rows()).noalias() += (1.0 - gamma) * s_filter.getD().transpose() * L_system.dfdux;
    L.dfdux.rightCols(x_filter.rows()).noalias() = (1.0 - gamma) * s_filter.getD().transpose() * dfduu_C;
  }

  return L;
}

}  // namespace ocs2
