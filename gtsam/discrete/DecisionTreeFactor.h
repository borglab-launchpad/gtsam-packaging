/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DecisionTreeFactor.h
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Ordering.h>

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

  class DiscreteConditional;

  /**
   * A discrete probabilistic factor
   */
  class GTSAM_EXPORT DecisionTreeFactor : public DiscreteFactor,
                                          public AlgebraicDecisionTree<Key> {
   public:
    // typedefs needed to play nice with gtsam
    typedef DecisionTreeFactor This;
    typedef DiscreteFactor Base;  ///< Typedef to base class
    typedef boost::shared_ptr<DecisionTreeFactor> shared_ptr;
    typedef AlgebraicDecisionTree<Key> ADT;

   protected:
    std::map<Key, size_t> cardinalities_;

   public:
    /// @name Standard Constructors
    /// @{

    /** Default constructor for I/O */
    DecisionTreeFactor();

    /** Constructor from DiscreteKeys and AlgebraicDecisionTree */
    DecisionTreeFactor(const DiscreteKeys& keys, const ADT& potentials);

    /** Constructor from doubles */
    DecisionTreeFactor(const DiscreteKeys& keys,
                      const std::vector<double>& table);

    /** Constructor from string */
    DecisionTreeFactor(const DiscreteKeys& keys, const std::string& table);

    /// Single-key specialization
    template <class SOURCE>
    DecisionTreeFactor(const DiscreteKey& key, SOURCE table)
        : DecisionTreeFactor(DiscreteKeys{key}, table) {}

    /// Single-key specialization, with vector of doubles.
    DecisionTreeFactor(const DiscreteKey& key, const std::vector<double>& row)
        : DecisionTreeFactor(DiscreteKeys{key}, row) {}

    /** Construct from a DiscreteConditional type */
    explicit DecisionTreeFactor(const DiscreteConditional& c);

    /// @}
    /// @name Testable
    /// @{

    /// equality
    bool equals(const DiscreteFactor& other, double tol = 1e-9) const override;

    // print
    void print(
        const std::string& s = "DecisionTreeFactor:\n",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override;

    /// @}
    /// @name Standard Interface
    /// @{

    /// Value is just look up in AlgebraicDecisonTree
    double operator()(const DiscreteValues& values) const override {
      return ADT::operator()(values);
    }

    /// multiply two factors
    DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
      return apply(f, ADT::Ring::mul);
    }

    static double safe_div(const double& a, const double& b);

    size_t cardinality(Key j) const { return cardinalities_.at(j); }

    /// divide by factor f (safely)
    DecisionTreeFactor operator/(const DecisionTreeFactor& f) const {
      return apply(f, safe_div);
    }

    /// Convert into a decisiontree
    DecisionTreeFactor toDecisionTreeFactor() const override { return *this; }

    /// Create new factor by summing all values with the same separator values
    shared_ptr sum(size_t nrFrontals) const {
      return combine(nrFrontals, ADT::Ring::add);
    }

    /// Create new factor by summing all values with the same separator values
    shared_ptr sum(const Ordering& keys) const {
      return combine(keys, ADT::Ring::add);
    }

    /// Create new factor by maximizing over all values with the same separator.
    shared_ptr max(size_t nrFrontals) const {
      return combine(nrFrontals, ADT::Ring::max);
    }

    /// Create new factor by maximizing over all values with the same separator.
    shared_ptr max(const Ordering& keys) const {
      return combine(keys, ADT::Ring::max);
    }

    /// @}
    /// @name Advanced Interface
    /// @{

    /**
     * Apply binary operator (*this) "op" f
     * @param f the second argument for op
     * @param op a binary operator that operates on AlgebraicDecisionTree
     */
    DecisionTreeFactor apply(const DecisionTreeFactor& f, ADT::Binary op) const;

    /**
     * Combine frontal variables using binary operator "op"
     * @param nrFrontals nr. of frontal to combine variables in this factor
     * @param op a binary operator that operates on AlgebraicDecisionTree
     * @return shared pointer to newly created DecisionTreeFactor
     */
    shared_ptr combine(size_t nrFrontals, ADT::Binary op) const;

    /**
     * Combine frontal variables in an Ordering using binary operator "op"
     * @param nrFrontals nr. of frontal to combine variables in this factor
     * @param op a binary operator that operates on AlgebraicDecisionTree
     * @return shared pointer to newly created DecisionTreeFactor
     */
    shared_ptr combine(const Ordering& keys, ADT::Binary op) const;

    /// Enumerate all values into a map from values to double.
    std::vector<std::pair<DiscreteValues, double>> enumerate() const;

    /// Return all the discrete keys associated with this factor.
    DiscreteKeys discreteKeys() const;

    /// @}
    /// @name Wrapper support
    /// @{

    /** output to graphviz format, stream version */
    void dot(std::ostream& os,
            const KeyFormatter& keyFormatter = DefaultKeyFormatter,
            bool showZero = true) const;

    /** output to graphviz format, open a file */
    void dot(const std::string& name,
            const KeyFormatter& keyFormatter = DefaultKeyFormatter,
            bool showZero = true) const;

    /** output to graphviz format string */
    std::string dot(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                    bool showZero = true) const;

    /**
     * @brief Render as markdown table
     *
     * @param keyFormatter GTSAM-style Key formatter.
     * @param names optional, category names corresponding to choices.
     * @return std::string a markdown string.
     */
    std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                        const Names& names = {}) const override;

    /**
     * @brief Render as html table
     *
     * @param keyFormatter GTSAM-style Key formatter.
     * @param names optional, category names corresponding to choices.
     * @return std::string a html string.
     */
    std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                    const Names& names = {}) const override;

    /// @}
  };

// traits
template <>
struct traits<DecisionTreeFactor> : public Testable<DecisionTreeFactor> {};

}  // namespace gtsam
