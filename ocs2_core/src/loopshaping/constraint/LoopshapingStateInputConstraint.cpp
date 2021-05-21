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
#include <ocs2_core/loopshaping/constraint/LoopshapingStateInputConstraint.h>

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintInputPattern.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LoopshapingStateInputConstraint::getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
  assert(dynamic_cast<const LoopshapingPreComputation*>(&preComp) != nullptr);
  const LoopshapingPreComputation& preCompLS = *reinterpret_cast<const LoopshapingPreComputation*>(&preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& preComp_system = preCompLS.getSystemPreComputation();

  return systemConstraint_->getValue(t, x_system, u_system, preComp_system);
}

}  // namespace ocs2
