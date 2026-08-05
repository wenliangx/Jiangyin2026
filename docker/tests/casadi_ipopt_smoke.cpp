#include <casadi/casadi.hpp>

#include <cmath>
#include <iostream>

int main() {
  using namespace casadi;

  SX x = SX::sym("x");
  SXDict nlp{{"x", x}, {"f", (x - 3) * (x - 3)}};
  Dict options{{"ipopt.print_level", 0}, {"print_time", false}};
  Function solver = nlpsol("solver", "ipopt", nlp, options);
  DMDict result = solver(DMDict{{"x0", 0}});
  const double solution = static_cast<double>(result.at("x"));

  std::cout << "CasADi/IPOPT solution=" << solution << std::endl;
  return std::abs(solution - 3.0) < 1e-7 ? 0 : 1;
}
