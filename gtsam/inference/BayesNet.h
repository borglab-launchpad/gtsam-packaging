/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file    BayesNet.h
 * @brief   Bayes network
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>

#include <boost/shared_ptr.hpp>
#include <string>

namespace gtsam {

/**
 * A BayesNet is a tree of conditionals, stored in elimination order.
 * @addtogroup inference
 */
template <class CONDITIONAL>
class BayesNet : public FactorGraph<CONDITIONAL> {
 private:
  typedef FactorGraph<CONDITIONAL> Base;

 public:
  typedef typename boost::shared_ptr<CONDITIONAL>
      sharedConditional;  ///< A shared pointer to a conditional

 protected:
  /// @name Standard Constructors
  /// @{

  /** Default constructor as an empty BayesNet */
  BayesNet() {}

  /** Construct from iterator over conditionals */
  template <typename ITERATOR>
  BayesNet(ITERATOR firstConditional, ITERATOR lastConditional)
      : Base(firstConditional, lastConditional) {}

  /// @}

 public:
  /// @name Testable
  /// @{

  /** print out graph */
  void print(
      const std::string& s = "BayesNet",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// @}

  /// @name Graph Display
  /// @{

  /// Output to graphviz format, stream version.
  void dot(std::ostream& os,
           const KeyFormatter& keyFormatter = DefaultKeyFormatter,
           const DotWriter& writer = DotWriter()) const;

  /// Output to graphviz format string.
  std::string dot(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                  const DotWriter& writer = DotWriter()) const;

  /// output to file with graphviz format.
  void saveGraph(const std::string& filename,
                 const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                 const DotWriter& writer = DotWriter()) const;

  /// @}
};

}  // namespace gtsam

#include <gtsam/inference/BayesNet-inst.h>
