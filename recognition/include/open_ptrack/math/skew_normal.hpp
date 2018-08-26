#ifndef SKEW_NORMAL_HPP
#define SKEW_NORMAL_HPP

#include <cmath>

/**
 * @brief the probability density function of a skew normal distribution
 * @param alpha  skewness
 * @param zeta   location (mean)
 * @param omega  scale (variance)
 * @param x      variable
 * @return the probability density
 * @ref https://en.wikipedia.org/wiki/Skew_normal_distribution
 */
inline double skew_normal_distribution(double alpha, double zeta, double omega, double x) {
  x = (x - zeta) / omega;

  double phi = 1.0 / (sqrt(2.0 * M_PI)) * exp( -x*x / 2.0 );
  double PHI = 1.0 / 2.0 * (1.0 + erf(alpha * x / M_SQRT2));
  return 2.0 / omega * phi * PHI;
}

/**
 * @brief The SkewNormalDistribution class
 */
class SkewNormalDistribution {
public:
  /**
   * @brief constructor
   * @param alpha  skewness
   * @param zeta   location (mean)
   * @param omega  scale (variance)
   */
  SkewNormalDistribution(double alpha, double zeta, double omega)
    :alpha(alpha), zeta(zeta), omega(omega)
  {}

  double prob(double x) const {
    return skew_normal_distribution(alpha, zeta, omega, x);
  }

  double operator() (double x) const {
    return prob(x);
  }

private:
  double alpha, zeta, omega;
};

#endif // SKEW_NORMAL_HPP
