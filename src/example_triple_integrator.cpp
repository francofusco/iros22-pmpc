#include <matplotlibcpp.h>
#include <pmpc/controls/simple.hpp>
#include <pmpc/controls/zoh.hpp>
#include <pmpc/controls/lerp.hpp>
#include <pmpc/controls/function_basis.hpp>
#include <pmpc/models/continuous/integrator.hpp>
#include <pmpc/models/discrete/linear_discretized.hpp>
#include <pmpc/models/discrete/euler_integrator.hpp>
#include <pmpc/predictors/linear.hpp>
#include <pmpc/optimization/quadratic_objective.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>
#include <pmpc/optimization/simple_control_constraints.hpp>
#include <pmpc/optimization/lerp_control_constraints.hpp>
#include <pmpc/optimization/function_basis_constraints.hpp>
#include <pmpc/optimization/linear_solver_qpoases.hpp>
#include <chrono>
#include <memory>
#include "utils.hpp"


int main(int argc, char **argv) {
  // some very simple settings
  const std::string FULL_NAME("full");
  const std::string SIMPLE_NAME("simple");
  const std::string ZOH_NAME("zoh");
  const std::string LERP_NAME("lerp");
  const std::string POLY_NAME("poly");
  std::string param_name(SIMPLE_NAME);

  // function that prints the usage of this program
  auto print_usage = [&]() {
    std::cout << "Usage: " << argv[0] << " [parameterization]" << std::endl;
    std::cout << "parameterization: one of the following" << std::endl;
    std::cout << "  " << FULL_NAME << std::endl;
    std::cout << "  " << SIMPLE_NAME << std::endl;
    std::cout << "  " << ZOH_NAME << std::endl;
    std::cout << "  " << LERP_NAME << std::endl;
    std::cout << "  " << POLY_NAME << std::endl;
  };

  // process the first argument to allow the use of different parameterizations
  if(argc > 1) {
    std::string arg2(argv[1]);
    if(arg2 == "-h" || arg2 == "--help") {
      print_usage();
      return 0;
    }
    else {
      param_name = arg2;
    }
  }

  // some local definitions
  namespace plt = matplotlibcpp;
  using namespace pmpc;
  using namespace pmpc::models;
  PMPC_MATRIX_VECTOR_TYPEDEFS(double);
  // Define the system
  const double dt = 0.01; // prediction sampling time
  const double Thor = 2.0; // prediction horizon, in seconds
  const int Np = std::round(Thor/dt); // Number of prediction samples
  std::vector<int> free_samples = range<int>(0, Np, Np/5);
  int Nc = free_samples.size();
  continuous::Integrator<double> linear_integrator(3);
  discrete::LinearDiscretized<double> model(dt, linear_integrator);

  std::unique_ptr<controls::Linear<double>> control_ptr;
  std::unique_ptr<optimization::LinearControlConstraints<double>> control_constraints_ptr;

  if(param_name == FULL_NAME) {
    control_ptr.reset(new controls::Simple<double>(model.dim_u, Np, Np));
    control_constraints_ptr.reset(new optimization::SimpleControlConstraints<double>(static_cast<controls::Simple<double>&>(*control_ptr)));
  }
  else if(param_name == SIMPLE_NAME) {
    control_ptr.reset(new controls::Simple<double>(model.dim_u, Np, Nc));
    control_constraints_ptr.reset(new optimization::SimpleControlConstraints<double>(static_cast<controls::Simple<double>&>(*control_ptr)));
  }
  else if(param_name == ZOH_NAME) {
    control_ptr.reset(new controls::ZOH<double>(model.dim_u, Np, free_samples));
    control_constraints_ptr.reset(new optimization::LinearControlConstraints<double>(static_cast<controls::ZOH<double>&>(*control_ptr)));
  }
  else if(param_name == LERP_NAME) {
    control_ptr.reset(new controls::LERP<double>(model.dim_u, Np, free_samples));
    control_constraints_ptr.reset(new optimization::LERPControlConstraints<double>(static_cast<controls::LERP<double>&>(*control_ptr)));
  }
  else if(param_name == POLY_NAME) {
    control_ptr.reset(controls::FunctionBasis<double>::DampedPolyPtr(model.dim_u, Np, Nc, true, dt, Thor/(Nc-1)));
    unsigned int Nconstr = std::max(10, 2*Nc);
    std::vector<int> constrained_samples = range<int>(0, Np, Np/Nconstr);
    control_constraints_ptr.reset(new optimization::FunctionBasisConstraints<double>(static_cast<controls::FunctionBasis<double>&>(*control_ptr), constrained_samples));
  }
  else {
    std::cout << "Sorry, the parameterization name '" << param_name << "' seems invalid." << std::endl;
    print_usage();
    return 0;
  }

  // "convert" the pointers into references - I simply like the "." access more than the "->" one :P
  controls::Linear<double>& control = *control_ptr;
  optimization::LinearControlConstraints<double>& control_constraints = *control_constraints_ptr;

  // constrain the control samples
  const double umax = 1.0;
  control_constraints.addBounds({-umax}, {umax});
  control_constraints.addVariationBounds({10*dt*umax});

  // create the predictor and instanciate the solver
  predictors::Linear<double> predictor(control, model);
  optimization::QuadraticObjective<double> objective(predictor);
  objective.setStateWeights({1e3, 0, 0});
  objective.setControlWeights({1e-3});
  objective.setDesiredStates({1, 0, 0});
  optimization::LinearSolverQPOASES solver(predictor, objective);
  solver.setControlConstraintsInstance(control_constraints);

  VectorXs xref = VectorXs::Zero(model.dim_x * control.np);
  VectorXs x0 = VectorXs::Zero(model.dim_x);
  VectorXs xnew = VectorXs::Zero(model.dim_x);
  VectorXs u = VectorXs::Zero(model.dim_u);
  VectorXs p = VectorXs::Zero(control.dim_p);

  const double omega = 0.5;
  const double amplitude = 0.5;
  auto get_ref = [&](double t) { return sin(omega*t+1e-10) > 0 ? amplitude : -amplitude; };

  double t = 0.0;
  std::vector<double> tsim;
  std::vector<double> pos;
  std::vector<double> pos_ref;
  std::vector<double> command;
  const double T = 20.0;

  std::vector<double> topt;
  topt.reserve(T/dt);
  std::cout << "Starting optimization" << std::endl;
  unsigned int it = 0;
  char progress[] = {'|', '\\', '-', '/'};

  while(t < T) {
    tsim.push_back(t);
    pos.push_back(x0(0));
    pos_ref.push_back(get_ref(t));

    for(unsigned int i=0; i<control.np; i++) {
      xref(model.dim_x*i) = get_ref(t + (i+1)*dt);
    }
    objective.setDesiredStates(xref);

    // Some fancy progress bar ;)
    constexpr auto Nchars = 20;
    std::cout << "\r" << progress[it++%4] << " optimizing... " << progressbar(std::round(Nchars*t/T), Nchars, '*') << std::flush;

    auto start = std::chrono::high_resolution_clock::now();
    if(!solver.solve(x0, p, true)) {
      std::cout << std::endl;
      std::cout << "Failed to solve the MPC optimization" << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    topt.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count()/1e3);

    control.controlSample(p, 0, u);
    command.push_back(u(0));
    model.function(x0, u, xnew);
    x0 = xnew;
    t += dt;
  }

  const unsigned int SKIP = 10;
  double topt_max = *std::max_element(topt.begin()+SKIP, topt.end());
  auto topt_sat = topt;
  for(int i=0; i<SKIP; i++)
    topt_sat[i] = std::min(topt_sat[i], topt_max);

  std::cout << std::endl;
  std::cout << "Used parameterization: " << param_name << " (" << control.dim_p << " parameters)" << std::endl;
  std::cout << "Average optimization time: " << average(topt_sat) << "us" << std::endl;

  plt::subplot(2,1,1);
  plt::named_plot("reference", tsim, pos_ref, "r");
  plt::named_plot("position", tsim, pos, "k");
  plt::named_plot("command", tsim, command, "b");
  plt::legend();
  plt::subplot(2,1,2);
  plt::named_plot("opt.time [us]", tsim, topt_sat);
  plt::legend();
  plt::show();

  return 0;
}
