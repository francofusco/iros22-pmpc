#include <pmpc/controls/simple.hpp>
#include <pmpc/controls/zoh.hpp>
#include <pmpc/controls/lerp.hpp>
#include <pmpc/controls/laguerre.hpp>
#include <pmpc/controls/function_basis.hpp>
#include <pmpc/models/continuous/crane_pendulum.hpp>
#include <pmpc/models/discrete/euler_integrator.hpp>
#include <pmpc/models/discrete/rk4_integrator.hpp>
#include <pmpc/predictors/recursive.hpp>
#include <pmpc/optimization/quadratic_objective.hpp>
#include <pmpc/optimization/linear_control_constraints.hpp>
#include <pmpc/optimization/simple_control_constraints.hpp>
#include <pmpc/optimization/lerp_control_constraints.hpp>
#include <pmpc/optimization/laguerre_control_constraints.hpp>
#include <pmpc/optimization/function_basis_constraints.hpp>
#include <pmpc/optimization/parameters_initialization.hpp>
#include <pmpc/optimization/simplified_gauss_newton_solver.hpp>
#include <chrono>
#include <fstream>
#include <memory>
#include <map>
#include "utils.hpp"



bool runTest(
  const std::string& program_name,
  const std::string& param_name,
  const int num_params,
  const double dt,
  std::vector<double>& topt,
  std::vector<std::vector<double>>& X,
  std::vector<std::vector<double>>& U,
  std::vector<std::vector<double>>& Xref,
  std::vector<double>& obj
)
{
  // some very simple settings
  const std::string FULL_NAME("full");
  const std::string SIMPLE_NAME("simple");
  const std::string ZOH_NAME("zoh");
  const std::string LERP_NAME("lerp");
  const std::string POLY_NAME("poly");

  // funtion that prints the usage of this program
  auto print_usage = [&]() {
    std::cout << "Usage: " << program_name << " [parameterization] [number_of_parameters]" << std::endl;
    std::cout << "parameterization: one of the following" << std::endl;
    std::cout << "  " << FULL_NAME << std::endl;
    std::cout << "  " << SIMPLE_NAME << std::endl;
    std::cout << "  " << ZOH_NAME << std::endl;
    std::cout << "  " << LERP_NAME << std::endl;
    std::cout << "  " << POLY_NAME << std::endl;
  };

  // give help if needed
  if(
      param_name == "" ||
      param_name == "help" ||
      param_name == "-h" ||
      param_name == "--help"
  )
  {
    print_usage();
    return false;
  }

  // some local definitions
  using namespace pmpc;
  using namespace pmpc::models;
  PMPC_MATRIX_VECTOR_TYPEDEFS(double);
  // Define the system
  const double dtsim = dt; // control sampling time
  const double Thor = 1.0; // prediction horizon, in seconds
  const int Np = (int)(Thor/dt); // Number of prediction samples
  std::vector<int> free_samples = range<int>(0, Np, Np/num_params);
  if(free_samples.size() > num_params)
    free_samples.pop_back();
  const int Nc = free_samples.size();
  continuous::CranePendulum<double> crane(0.25, 1.178e-1, 1.250e-3, 0.3, M_PI);
  discrete::EulerIntegrator<double> model(dt, crane);

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
    // std::vector<int> free_samples = {0, 2, 5, 10, 14};
    control_ptr.reset(new controls::LERP<double>(model.dim_u, Np, free_samples));
    control_constraints_ptr.reset(new optimization::LERPControlConstraints<double>(static_cast<controls::LERP<double>&>(*control_ptr)));
  }
  else if(param_name == POLY_NAME) {
    // control_ptr.reset(new controls::Laguerre<double>(model.dim_u, Np, Nc, true, 1.0/3.0, dt));
    // control_ptr.reset(controls::FunctionBasis<double>::LaguerrePtr(model.dim_u, Np, Nc, true, dt, 1.0/3.0));
    control_ptr.reset(controls::FunctionBasis<double>::DampedPolyPtr(model.dim_u, Np, Nc, true, dt, Thor/(Nc-1)));
    unsigned int Nconstr = std::max(10, 2*Nc);
    std::vector<int> constrained_samples = range<int>(0, Np, Np/Nconstr);
    // control_constraints_ptr.reset(new optimization::LaguerreControlConstraints<double>(static_cast<controls::Laguerre<double>&>(*control_ptr), constrained_samples));
    control_constraints_ptr.reset(new optimization::FunctionBasisConstraints<double>(static_cast<controls::FunctionBasis<double>&>(*control_ptr), constrained_samples));
  }
  else {
    std::cout << "Sorry, the parameterization name '" << param_name << "' seems invalid." << std::endl;
    print_usage();
    return false;
  }

  // "convert" the pointers into references - I simply like the "." access more than the "->" one :P
  controls::Linear<double>& control = *control_ptr;
  optimization::LinearControlConstraints<double>& control_constraints = *control_constraints_ptr;

  // constraint the control samples
#ifdef SWINGUP
  const double UMAX = 12.0;
#else
  const double UMAX = 0.1;
#endif
  control_constraints.addBounds({-UMAX}, {UMAX});
  // control_constraints.addVariationBounds({0.02});
  control_constraints.update(VectorXs::Zero(control.dim_p));

  // create the predictor and instanciate the solver
  predictors::Recursive<double> predictor(control, model);
  optimization::QuadraticObjective<double> objective(predictor);
#ifdef SWINGUP
  objective.setStateWeights({1e3/Np, 1e3/Np, 1e-2/Np, 1e-2/Np});
  objective.setControlWeights({1e-2/Np});
#else
  objective.setStateWeights({200.0/Np, 50.0/Np, 7.0/Np, 2.0/Np});
  objective.setControlWeights({1.0/Np});
#endif
  // objective.setDesiredStates({0.35, 0.0, 0.0, 0.0});
  objective.setDesiredStates({0.0, 0.0, 0.0, 0.0});
  pmpc::optimization::SimplifiedGaussNewtonSolver solver(predictor, objective);
  solver.setControlConstraintsInstance(control_constraints);

  VectorXs xref = VectorXs::Zero(model.dim_x * control.np);
  VectorXs x0 = VectorXs::Zero(model.dim_x);
  // VectorXs x0(model.dim_x);
  // x0 << 0.35, 0.0, 0.0, 0.0;
#ifdef SWINGUP
  x0(1) = M_PI;
#endif
  VectorXs xnew = VectorXs::Zero(model.dim_x);
  VectorXs u = VectorXs::Zero(model.dim_u);
  VectorXs p = VectorXs::Zero(control.dim_p);

#ifdef SWINGUP
  VectorXs Useed10ms(100);
  Useed10ms << 11.999992818743964,11.99999153119238,11.999990062251634,11.999988424925439,11.99998657752102,11.999984183007026,11.999983429719416,11.999985842236503,11.999986904597375,11.999988561722503,11.999992967060233,11.999809936093163,-10.980720612696615,-11.999946719897634,-11.99996362018073,-11.999968453703184,-11.999969741411501,-11.999968985490673,-11.99996643088159,-11.999961720058765,-11.999953809898217,-11.999940356708672,-11.999915878994294,-11.999865825417087,-11.999743008525902,-11.999336129413235,-11.996892883714636,-11.880144056850048,0.05791742514907716,1.1278600187322898,1.0187785021564828,0.9094987599907316,0.8186167015149102,0.7440006031409794,0.6843660374518664,0.6382831122104254,0.6041431267418,0.5803256438656546,0.5652978939850062,0.5576346829493422,0.5560139187857379,0.5592155303291997,0.5661246596921395,0.5757344892494315,0.5871467914257048,0.5995703686465216,0.6123179504485353,0.6248019477638361,0.6365293102712207,0.6470956636228808,0.6561788746149183,0.6635321791005492,0.6689770023454908,0.6723956009308791,0.6737236557391271,0.672942944168345,0.6700742146709233,0.6651703771269192,0.6583101085395459,0.6495919557058124,0.6391289961215236,0.6270440967799447,0.6134657890896005,0.598524758227719,0.5823509277042889,0.5650711055431997,0.5468071475053937,0.5276745853149349,0.507781663622833,0.48722872810161977,0.46610790813980363,0.444503040506306,0.42248978462245457,0.4001358851713126,0.3775015433239826,0.35463986348142956,0.3315973479014822,0.3084144166838914,0.28512593522469026,0.26176173534246205,0.23834711981399093,0.21490334303717343,0.1914480629999664,0.16799576172306718,0.14455813291472944,0.12114443678755343,0.0977618228976553,0.07441562253537962,0.05110961268947038,0.02784625395787443,0.004626904964703128,-0.018547984553674592,-0.04167870229899038,-0.06476613183504785,-0.08781166055978336,-0.11081706338945703,-0.1337813409230585,-0.1565795760101222,-0.1742218133518949,8.401851979474649e-13;
  VectorXs Useed(control.np);
  if(dt == 0.01) {
    Useed = Useed10ms;
  }
  else {
    int factor = std::round(0.01 / dt);
    if(std::fabs(factor*dt-0.01) > 1e-6)
      std::cout << "WARNING: the given time sampling is not a sub-multiple of 0.01 and thus seed initialization might be incorrect" << std::endl;
    for(unsigned int k=0; k<Useed10ms.rows(); k++) {
      Useed.segment(k*factor, factor).setConstant(Useed10ms(k));
    }
  }

  optimization::initializeParameters<double>(control_constraints, Useed, p);
  control_constraints.addVariationBounds({UMAX*dt*3});
  optimization::initializeParameters<double>(control_constraints, Useed, p);
#endif

  auto get_ref = [](double t) { return 0.3*sin(2.5*t); };

  double t = 0.0;
  // const double T = 6.0;
#ifdef SWINGUP
  const double T = 3.0;
#else
  const double T = 30.0;
#endif

  auto asVector = [](const Eigen::Ref<const VectorXs>& v) {
    std::vector<double> vec(v.size());
    Eigen::Map<VectorXs>(vec.data(),v.size()) = v;
    return vec;
  };

  topt = std::vector<double>();
  topt.reserve(T/dt);
  X = std::vector<std::vector<double>>(model.dim_x);
  U = std::vector<std::vector<double>>(model.dim_u);
  Xref = std::vector<std::vector<double>>(model.dim_x);
  obj = std::vector<double>();
  obj.reserve(T/dt);

  for(int i=0; i<model.dim_x; i++)
    X[i].push_back(x0(i));

  unsigned int it = 0;

  while(t < T) {
#ifndef SWINGUP
    // Get reference trajectory
    for(int k=0; k<control.np; k++) {
      double tk = t + (k+1)*dt;
      if(tk < 7.5)
        xref(k*model.dim_x) = -0.5;
      else if(tk < 15)
        xref(k*model.dim_x) = 0.5;
      else
        xref(k*model.dim_x) = 0.2*std::sin(0.4*M_PI*tk);
    }
    objective.setDesiredStates(xref);
#endif

    // TEMP: this should be done by the solver
    control_constraints.update(p);

#ifndef BENCHMARK
    // Some fancy progress bar ;)
    constexpr auto Nchars = 20;
    std::cout << "\r" << progressChar(it++) << " optimizing... " << progressbar(std::round(Nchars*t/T), Nchars, '*') << std::flush;
#endif

    auto start = std::chrono::high_resolution_clock::now();
    if(!solver.solve(x0, p, true)) {
      std::cout << "  Failed to solve the MPC optimization: " << solver.lastError() << std::endl;
      return false;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    topt.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count()/1e3);

    double obj_val;
    objective.evaluate(x0, p, obj_val);
    obj.push_back(obj_val);

    control.controlSample(p, 0, u);
    models::discrete::RK4Integrator<double>::integrate(crane, x0, u, xnew, dtsim, 20);
    x0 = xnew;
    t += dtsim;
    for(int i=0; i<model.dim_x; i++)
      X[i].push_back(x0(i));
    for(int i=0; i<model.dim_u; i++)
      U[i].push_back(u(i));
    for(int i=0; i<model.dim_x; i++)
      Xref[i].push_back(xref(i));
  }

#ifndef BENCHMARK
  std::cout << std::endl;
#endif

  return true;
}


struct TestData {
  std::string name;
  std::vector<std::vector<double>> topt;
  std::vector<std::vector<std::vector<double>>> X;
  std::vector<std::vector<std::vector<double>>> Xref;
  std::vector<std::vector<std::vector<double>>> U;
  std::vector<std::vector<double>> obj;
};



#ifdef BENCHMARK

int main(int argc, char **argv) {
  const std::string param = argc > 1 ? argv[1] : "help";
  if(argc < 2 || param == "help" || param == "-h" || param == "--help") {
    std::cout << "Usage: " << argv[0] << " param [num_params] [repetitions] [dt_ms]" << std::endl;
    return 0;
  }
  const int num_params = argc > 2 ? std::atoi(argv[2]) : 5;
  const int repetitions = argc > 3 ? std::atoi(argv[3]) : 10;
  const double dt = argc > 4 ? std::atoi(argv[4])/1000.0 : 0.01;

  std::map<std::string,std::string> colors{
    {"simple", "[0.83921569, 0.15294118, 0.15686275]"},
    {"zoh", "[0.17254902, 0.62745098, 0.17254902]"},
    {"lerp", "[0.12156863, 0.46666667, 0.70588235]"},
    {"poly", "[0.0, 0.0, 0.0]"},
    {"full", "[0.7372549, 0.74117647, 0.13333333]"}
  };

  const std::string ind = "  ";
  const std::string ind2 = ind + ind;
  const std::string ind3 = ind2 + ind;
  const std::string ind4 = ind3 + ind;
  const std::string ind5 = ind4 + ind;
#ifdef SWINGUP
  std::string swingup("swingup-");
#else
  std::string swingup("");
#endif
  std::string param_name = param=="full" ? "full" : param + "-" + std::to_string(num_params);
  std::string results_filename = "pmpc-" + swingup + param_name + "-" + std::to_string((int)(dt*1000)) + "ms.json";

  TestData test;
  test.name = param_name;

  for(unsigned int i=0; i<repetitions; i++) {
    std::cout << "\r" << param << " " << num_params << " - " << (i+1) << " of " << repetitions << std::flush;
    std::vector<double> topt, obj;
    std::vector<std::vector<double>> X, U, Xref;
    if(!runTest(argv[0], param, num_params, dt, topt, X, U, Xref, obj))
      continue;
    test.topt.push_back(topt);
    test.X.push_back(X);
    test.U.push_back(U);
    test.Xref.push_back(Xref);
    test.obj.push_back(obj);
  }

  std::cout << std::endl;

  if(test.topt.size() == 0)
    return 0;

  std::ofstream json(results_filename);

  #define DUMP(x) {json << x << std::endl;};
  #define COMMA(idx,len) (idx+1==len ? "" : ",")

  DUMP("{");
  DUMP(ind << "\"" << test.name << "\": {");
  DUMP(ind2 << "\"color\": " << colors.at(param) << ",");

  // Topt
  DUMP(ind2 << "\"Topt\": [");
  for(int i=0; i<test.topt.size(); i++) {
    const auto& topt = test.topt[i];
    DUMP(ind3 << "[");
    for(int i=0; i<topt.size(); i++)
      DUMP(ind4 << topt[i] << COMMA(i,topt.size()));
    DUMP(ind3 << "]" << COMMA(i,test.topt.size()));
  }
  DUMP(ind2 << "],");

  // X
  DUMP(ind2 << "\"X\": [");
  for(int i=0; i<test.X.size(); i++) {
    DUMP(ind3 << "[");
    for(int j=0; j<test.X[i].size(); j++) {
      DUMP(ind4 << "[");
      for(int k=0; k<test.X[i][j].size(); k++) {
        DUMP(ind5 << test.X[i][j][k] << COMMA(k, test.X[i][j].size()));
      }
      DUMP(ind4 << "]" << COMMA(j, test.X[i].size()));
    }
    DUMP(ind3 << "]" << COMMA(i, test.X.size()));
  }
  DUMP(ind2 << "],");

  // U
  DUMP(ind2 << "\"U\": [");
  for(int i=0; i<test.U.size(); i++) {
    DUMP(ind3 << "[");
    for(int j=0; j<test.U[i].size(); j++) {
      DUMP(ind4 << "[");
      for(int k=0; k<test.U[i][j].size(); k++) {
        DUMP(ind5 << test.U[i][j][k] << COMMA(k, test.U[i][j].size()));
      }
      DUMP(ind4 << "]" << COMMA(j, test.U[i].size()));
    }
    DUMP(ind3 << "]" << COMMA(i, test.U.size()));
  }
  DUMP(ind2 << "],");

  // Xref
  DUMP(ind2 << "\"Xref\": [");
  for(int i=0; i<test.Xref.size(); i++) {
    DUMP(ind3 << "[");
    for(int j=0; j<test.Xref[i].size(); j++) {
      DUMP(ind4 << "[");
      for(int k=0; k<test.Xref[i][j].size(); k++) {
        DUMP(ind5 << test.Xref[i][j][k] << COMMA(k, test.Xref[i][j].size()));
      }
      DUMP(ind4 << "]" << COMMA(j, test.Xref[i].size()));
    }
    DUMP(ind3 << "]" << COMMA(i, test.Xref.size()));
  }
  DUMP(ind2 << "],");

  // obj
  DUMP(ind2 << "\"objective\": [");
  for(int i=0; i<test.obj.size(); i++) {
    const auto& obj = test.obj[i];
    DUMP(ind3 << "[");
    for(int i=0; i<obj.size(); i++)
      DUMP(ind4 << obj[i] << COMMA(i,obj.size()));
    DUMP(ind3 << "]" << COMMA(i,test.obj.size()));
  }
  DUMP(ind2 << "]");

  DUMP(ind << "}");
  DUMP("}");

  json.close();

  return 0;
}

#else

#include <matplotlibcpp.h>

int main(int argc, char **argv) {
  namespace plt = matplotlibcpp;
  const std::string param = argc > 1 ? argv[1] : "help";
  const int num_params = argc > 2 ? std::atoi(argv[2]) : 5;
  const double dt = argc > 3 ? std::atoi(argv[3])/1000.0 : 0.01;
  std::vector<double> topt, obj;
  std::vector<std::vector<double>> X, U, Xref;
  if(!runTest(argv[0], param, num_params, dt, topt, X, U, Xref, obj))
    return 0;

  std::cout << "Average optimization time: " << average(topt) << "us" << std::endl;

  auto iters_x = range<double>(0, X[0].size(), 1);
  auto iters_u = range<double>(0, U[0].size(), 1);
  auto iters_xref = range<double>(1, Xref[0].size()+1, 1);
  auto iters_topt = range<double>(0, topt.size(), 1);
  auto iters_obj = range<double>(0, obj.size(), 1);

  plt::subplot(5,1,1);
  plt::named_plot("position", iters_x, X[0], "k");
  plt::named_plot("reference", iters_xref, Xref[0], "--r");
  plt::legend();
  plt::subplot(5,1,2);
  plt::named_plot("angle", iters_x, X[1], "k");
  plt::legend();
  plt::subplot(5,1,3);
  plt::named_plot("command", iters_u, U[0], "b");
  plt::legend();
  plt::subplot(5,1,4);
  plt::named_plot("topt", iters_topt, topt, "k");
  plt::legend();
  plt::subplot(5,1,5);
  plt::named_plot("objective", iters_obj, obj, "k");
  plt::legend();
  plt::show();

  return 0;
}
#endif
