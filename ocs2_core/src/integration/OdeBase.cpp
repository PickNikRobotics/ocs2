/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/integration/OdeBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OdeBase::OdeBase() : numFunctionCalls_(0) {
  modelDataArray_.reserve(DEFAULT_MODEL_DATA_CACHE_SIZE);
  systemFunction_ = [this](const vector_t& x, vector_t& dxdt, scalar_t t) {
    numFunctionCalls_++;
    computeFlowMap(t, x, dxdt);
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OdeBase::system_func_t OdeBase::systemFunction() {
  return systemFunction_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int OdeBase::getNumFunctionCalls() const {
  return numFunctionCalls_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdeBase::resetNumFunctionCalls() {
  numFunctionCalls_ = 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModelDataBase::array_t::iterator OdeBase::beginModelDataIterator() {
  return modelDataArray_.begin();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModelDataBase::array_t::iterator OdeBase::endModelDataIterator() {
  return modelDataArray_.end();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModelDataBase& OdeBase::modelDataEmplaceBack() {
  modelDataArray_.emplace_back();
  return modelDataArray_.back();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdeBase::clearModelDataArray() {
  modelDataArray_.clear();  // keeps allocated storage
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdeBase::computeJumpMap(const scalar_t& time, const vector_t& state, vector_t& mappedState) {
  mappedState = state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdeBase::computeGuardSurfaces(const scalar_t& time, const vector_t& state, vector_t& guardSurfacesValue) {
  guardSurfacesValue = -vector_t::Ones(1);
}
}  // namespace ocs2
