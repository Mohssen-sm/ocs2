/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#pragma once

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_ballbot_example/definitions.h"

namespace ocs2 {
namespace ballbot {

class BallbotCost final : public ocs2::QuadraticCostFunction {
 public:
  using BASE = ocs2::QuadraticCostFunction;

  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(x-x_{nominal})' Q (x-x_{nominal}) + 0.5(u-u_{nominal})' R (u-u_{nominal}) \f$
   * - \f$ \Phi = 0.5(x-x_{final})' Q_{final} (x-x_{final}) \f$.
   * @param [in] Q: \f$ Q \f$
   * @param [in] R: \f$ R \f$
   * @param [in] xNominal: \f$ x_{nominal}\f$
   * @param [in] uNominal: \f$ u_{nominal}\f$
   * @param [in] QFinal: \f$ Q_{final}\f$
   * @param [in] xFinal: \f$ x_{final}\f$
   */
  BallbotCost(const matrix_t& Q, const matrix_t& R, const vector_t& x_nominal, const vector_t& u_nominal, const matrix_t& Q_final,
              const vector_t& x_final)
      : QuadraticCostFunction(Q, R, x_nominal, u_nominal, Q_final, x_final) {}

  /**
   * Destructor
   */
  ~BallbotCost() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  BallbotCost* clone() const override { return new BallbotCost(*this); }

  /**
   * Sets the current time, state, and control input.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state vector.
   * @param [in] u: Current input vector.
   */
  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override {
    vector_t xNominal = BASE::costDesiredTrajectoriesPtr_->getDesiredState(t);
    vector_t uNominal = BASE::costDesiredTrajectoriesPtr_->getDesiredInput(t);

    // set base class
    BASE::setCurrentStateAndControl(t, x, u, xNominal, uNominal, xNominal);
  }
};

}  // namespace ballbot
}  // namespace ocs2
