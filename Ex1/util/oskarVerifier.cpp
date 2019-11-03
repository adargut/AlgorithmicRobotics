#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace fi = boost::filesystem;

typedef std::vector<fi::path>           Paths;
typedef std::vector<std::string>        String_array;
typedef std::vector<size_t>             Uint_array;

static const size_t s_def_verbose_level(0);
static const fi::path s_def_input_path("./");
static const std::string s_def_config_filename("oskar.cfg");
static const std::string s_def_solution_filename("solution.txt");
static const Uint_array s_def_dimensions({11, 11, 11});
static const Uint_array s_def_face0({
    1,1,1,1,1,1,1,1,1,1,1,
    1,0,0,0,0,0,0,0,0,0,1,
    1,1,1,0,1,1,1,1,1,0,1,
    1,0,1,0,0,0,0,0,1,0,1,
    1,0,1,0,1,1,1,1,1,1,1,
    1,0,0,0,1,0,0,0,1,0,1,
    1,0,1,1,1,0,1,1,1,0,1,
    1,0,0,0,1,0,1,0,1,0,1,
    1,1,1,0,1,0,1,0,1,0,1,
    1,0,0,0,0,0,0,0,0,0,1,
    1,1,1,1,1,1,1,1,1,1,1
});

static const Uint_array s_def_face1({
    1,1,1,1,1,1,1,1,1,1,1,
    1,0,0,0,0,0,0,0,1,0,1,
    1,0,1,0,1,1,1,0,1,0,1,
    1,0,1,0,1,0,0,0,0,0,1,
    1,1,1,0,1,0,1,1,1,1,1,
    1,0,1,0,1,0,1,0,0,0,1,
    1,0,1,0,1,0,1,0,1,0,1,
    1,0,0,0,1,0,0,0,1,0,1,
    1,0,1,1,1,1,1,0,1,0,1,
    1,0,1,0,0,0,0,0,1,0,1,
    1,1,1,1,1,1,1,1,1,1,1
});

static const Uint_array s_def_face2({
    1,1,1,1,1,1,1,1,1,1,1,
    1,0,0,0,1,0,0,0,0,0,1,
    1,0,1,1,1,0,1,1,1,0,1,
    1,0,0,0,1,0,1,0,0,0,1,
    1,1,1,0,1,1,1,0,1,0,1,
    1,0,1,0,0,0,0,0,1,0,1,
    1,0,1,0,1,1,1,1,1,0,1,
    1,0,1,0,0,0,1,0,0,0,1,
    1,0,1,1,1,1,1,0,1,1,1,
    1,0,0,0,0,0,0,0,0,0,1,
    1,1,1,1,1,1,1,1,1,1,1
});

namespace std {

  template <typename OutputStream>
  OutputStream& operator<<(OutputStream& os, const Paths& paths)
  {
    for (const auto& path : paths) {
      os << path.string();
      if (path != paths.back()) os << " ";
    }
    return os;
  }

  template <typename OutputStream>
  OutputStream& operator<<(OutputStream& os, const Uint_array& face)
  {
    size_t size(0);
    for (const auto& index : face) {
      os << std::to_string(index);
      if (++size != face.size()) os << ",";
    }
    return os;
  }
}

bool operator==(const Uint_array& a, const Uint_array& b)
{
  if (a.size() != b.size()) return false;
  size_t i(0);
  for (i = 0; i < a.size(); ++i) if (a[i] != b[i]) return false;
  return true;
}

// Uint_array start_pos = {3, 7, 7};
// Uint_array end_pos = {5, 1, 1};

//! Main entry point.
int main(int argc, char* argv[])
{
  po::options_description config_opts("Options");
  config_opts.add_options()
    ("quite,q", po::value<bool>()->default_value(true),
     "quite mode")
    ("verbose-level,v", po::value<size_t>()->default_value(0),
     "verbose level")
    ("input-path", po::value<Paths>()->composing()->
     default_value({s_def_input_path}),
     "input path")
    ("input-file", po::value<String_array>(), "input file")

    ("dimensions", po::value<Uint_array>()->default_value(s_def_dimensions),
     "dimensions")
    ("face0", po::value<Uint_array>()->default_value(s_def_face0), "face 0")
    ("face1", po::value<Uint_array>()->default_value(s_def_face1), "face 1")
    ("face2", po::value<Uint_array>()->default_value(s_def_face2), "face 2")
    ;

  po::positional_options_description positional_opts;
  positional_opts.add("input-file", -1);

  po::options_description opts;
  opts.add(config_opts);
  opts.add_options()("help", "print help message");

  po::variables_map variable_map;
  po::store(po::command_line_parser(argc, argv).options(opts)
            .positional(positional_opts).run(),
            variable_map);

  auto& config_fullname = s_def_config_filename;
  if (! config_fullname.empty()) {
    std::ifstream ifs;
    ifs.open(config_fullname);
    if (ifs.is_open()) {
      po::store(po::parse_config_file(ifs, config_opts), variable_map);
      ifs.close();
    }
  }
  po::notify(variable_map);

  if (variable_map.count("help")) {
    std::cout << opts << std::endl;
    return 0;
  }

  auto verbose_level = variable_map["verbose-level"].as<size_t>();
  // std::cout << "Verbose level: " << verbose_level << std::endl;

  auto num_input_files = (variable_map.count("input-file")) ?
    variable_map["input-file"].as<String_array>().size() : 0;
  // std::cout << "# input files: " << num_input_files << std::endl;

  const auto& solution_filename =
    (num_input_files == 0) ?
     s_def_solution_filename : variable_map["input-file"].as<String_array>()[0];
  if (verbose_level > 0)
    std::cout << "input file: " << solution_filename << std::endl;

  const auto& dims = variable_map["dimensions"].as<Uint_array>();
  const auto& face0 = variable_map["face0"].as<Uint_array>();
  const auto& face1 = variable_map["face1"].as<Uint_array>();
  const auto& face2 = variable_map["face2"].as<Uint_array>();

  if (verbose_level > 0) {
    std::cout << "Face 0 : " << face0.size() << ", " << face0 << std::endl;
    std::cout << "Face 1 : " << face1.size() << ", " << face1 << std::endl;
    std::cout << "Face 2 : " << face2.size() << ", " << face2 << std::endl;
  }

  std::ifstream is;
  is.open(solution_filename);
  if (! is.is_open()) return EXIT_FAILURE;
  std::string line1;
  std::getline(is, line1);
  std::string line2;
  std::getline(is, line2);
  std::string line3;
  std::getline(is, line3);
  is.close();

  // Start position
  Uint_array start_pos;
  start_pos.reserve(3);
  std::stringstream ss1(line1);
  std::copy(std::istream_iterator<size_t>(ss1), std::istream_iterator<size_t>(),
            std::back_inserter(start_pos));
  start_pos.shrink_to_fit();
  if (verbose_level > 0)
    std::cout << "Star: " << start_pos.size() << ", " << start_pos << std::endl;
  if (start_pos.size() != 3) {
    std::cerr << "Start position (" << start_pos <<") is not 3-dimensional!"
              << std::endl;
    return EXIT_FAILURE;
  }

  // End position
  Uint_array end_pos;
  end_pos.reserve(3);
  std::stringstream ss2(line2);
  std::copy(std::istream_iterator<size_t>(ss2), std::istream_iterator<size_t>(),
            std::back_inserter(end_pos));
  end_pos.shrink_to_fit();
  if (verbose_level > 0)
    std::cout << "End : " << end_pos.size() << ", " << end_pos << std::endl;
  if (end_pos.size() != 3) {
    std::cerr << "End position (" << end_pos <<") is not 3-dimensional!"
              << std::endl;
    return EXIT_FAILURE;
  }

  auto x = start_pos[0];
  auto y = start_pos[1];
  auto z = start_pos[2];
  if ((dims[0] <= x) || (dims[1] <= y) || (dims[2] <= z)) {
    std::cerr << "Start position out of boundary (" << start_pos << ")!"
              << std::endl;
    return EXIT_SUCCESS;
  }
  auto i1 = dims[1] * y + x;
  auto i2 = dims[2] * z + y;
  auto i3 = dims[0] * x + z;
  if ((face0[i1] == 1) || (face1[i2] == 1) || (face2[i3] == 1)) {
    std::cout << "Invalid start position (" << start_pos << ")!" << std::endl;
    return EXIT_SUCCESS;
  }

  x = end_pos[0];
  y = end_pos[1];
  z = end_pos[2];
  if ((dims[0] <= x) || (dims[1] <= y) || (dims[2] <= z)) {
    std::cerr << "End position out of boundary (" << end_pos << ")!"
              << std::endl;
    return EXIT_SUCCESS;
  }
  i1 = dims[1] * y + x;
  i2 = dims[2] * z + y;
  i3 = dims[0] * x + z;
  if ((face0[i1] == 1) || (face1[i2] == 1) || (face2[i3] == 1)) {
    std::cout << "Invalid end position (" << end_pos << ")!" << std::endl;
    return EXIT_SUCCESS;
  }

  // Solution
  Uint_array solution;
  std::stringstream ss3(line3);
  std::copy(std::istream_iterator<size_t>(ss3),
            std::istream_iterator<size_t>(),
            std::back_inserter(solution));
  solution.shrink_to_fit();
  if (verbose_level > 0)
    std::cout << "Solu: " << solution.size() << ", " << solution << std::endl;
  if (solution.empty()) {
    std::cerr << "Solution (" << solution <<") is empty!" << std::endl;
    return EXIT_FAILURE;
  }

  // Iterate
  Uint_array cur_pos = start_pos;
  size_t step(0);
  for (; step < solution.size(); ++step) {
    auto move = solution[step];
    // std::cout << "Move: " << move << std::endl;
    // std::cout << "Pos: " << cur_pos << std::endl;

    auto x = cur_pos[0];
    auto y = cur_pos[1];
    auto z = cur_pos[2];

    if (move == 0) {
      if ((x+1) == dims[0]) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[1] * y + x + 1;
      auto i2 = dims[0] * (x+1) + z;
      if (((face0[i1] == 0) && (face2[i2] == 0))) {
        cur_pos[0] = x + 1;
        continue;
      }
      break;
    }

    if (move == 1) {
      if (x == 0) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[1] * y + x - 1;
      auto i2 = dims[0] * (x-1) + z;
      if ((face0[i1] == 0) && (face2[i2] == 0)) {
        cur_pos[0] = x - 1;
        continue;
      }
      break;
    }

    if (move == 2) {
      if ((y+1) == dims[1]) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[2] * z + y + 1;
      auto i2 = dims[1] * (y + 1) + x;
      if ((face1[i1] == 0) && (face0[i2] == 0)) {
        cur_pos[1] = y + 1;
        continue;
      }
      break;
    }
    if(move == 3) {
      if (y == 0) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[2] * z + y - 1;
      auto i2 = dims[1] * (y - 1) + x;
      if ((face1[i1] == 0) && (face0[i2] == 0)) {
        cur_pos[1] = y - 1;
        continue;
      }
      break;
    }

    if (move == 4) {
      if ((z+1) == dims[2]) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[0] * x + z + 1;
      auto i2 = dims[2] * (z + 1) + y;
      if ((face2[i1] == 0) && (face1[i2] == 0)) {
        cur_pos[2] = z + 1;
        continue;
      }
      break;
    }

    if (move == 5) {
      if (z == 0) {
        std::cerr << "Attempt to access out of boundary!" << std::endl;
        break;
      }
      auto i1 = dims[0] * x + z - 1;
      auto i2 = dims[2] * (z - 1) + y;
      if ((face2[i1] == 0) && (face1[i2] == 0)) {
        cur_pos[2] = z - 1;
        continue;
      }
      break;
    }

    std::cerr << "Invalid move (" << move << ")!" << std::endl;
    return EXIT_FAILURE;
  }

  if (step != solution.size()) {
    std::cout << "Stopped after executing " << step << "steps." << std::endl;
  }

  if (cur_pos == end_pos) std::cout << "SUCCESS" << std::endl;
  else std::cout << "FAILURE (" << step << ")" << std::endl;

  std::exit(EXIT_SUCCESS);
}
