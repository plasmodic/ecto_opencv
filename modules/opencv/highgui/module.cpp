#include <boost/python.hpp>

#include <ecto/ecto.hpp>

#include "highgui.h"

ECTO_DEFINE_MODULE(highgui)
{
boost::python::enum_<ecto_opencv::Record::RecordCommands>("RecordCommands")
    .value("START", ecto_opencv::Record::START)
    .value("RESUME", ecto_opencv::Record::RESUME)
    .value("PAUSE", ecto_opencv::Record::PAUSE)
    .value("STOP", ecto_opencv::Record::STOP)
    .export_values()
    ;
}
