#include <nanobind/nanobind.h>

std::string hello_from_bin() { return "Hello from gtsam-nb!"; }

namespace nb = nanobind;
NB_MODULE(_core, m) {
  m.doc() = "nanobind hello module";

  m.def("hello_from_bin", &hello_from_bin, R"pbdoc(
      A function that returns a Hello string.
  )pbdoc");
}
