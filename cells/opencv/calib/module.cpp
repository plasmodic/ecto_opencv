#include <ecto/ecto.hpp>
#include "calib.hpp"
using namespace calib;
namespace bp = boost::python;
ECTO_DEFINE_MODULE(calib)
{
  bp::enum_<Pattern>("Pattern")
    .value("CHESSBOARD", CHESSBOARD)
    .value("CIRCLES_GRID", CIRCLES_GRID)
    .value("ASYMMETRIC_CIRCLES_GRID", ASYMMETRIC_CIRCLES_GRID)
    .export_values()
    ;

}
