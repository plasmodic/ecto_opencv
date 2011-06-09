#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
class DB;
class ObjectInstance;

struct Observation
{
  explicit Observation(ObjectInstance * s):session_(s){}
  template<typename Data>
  void add(const Data data, const std::string name)
  {

  }
  void commit()
  {

  }
  ObjectInstance* session_;
};

struct ObjectInstance
{
  explicit ObjectInstance(DB* db):db(db){

  }

  DB* db;
};

struct DB
{
  void connect(std::string collection_name)
  {

  }

};

using ecto::tendrils;

namespace
{
struct ObjectInserter
{
  static void declare_params(tendrils& params)
  {
  }

  static void declare_io(const tendrils& params, const tendrils& inputs,
                         tendrils& outputs)
  {

  }
  void configure(tendrils& params)
  {
    db_.connect("object_db");
    object_instance_ = ObjectInstance(&db_);
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    cv::Mat image = inputs.get<cv::Mat> ("image"), mask =
        inputs.get<cv::Mat> ("mask"), depth = inputs.get<cv::Mat> ("depth"), R =
        inputs.get<cv::Mat> ("R"), T = inputs.get<cv::Mat> ("T"), K =
        inputs.get<cv::Mat> ("K");

    Observation observation(&object_instance_);
    observation.add(image, "image");
    observation.add(mask, "mask");
    observation.add(depth, "depth");
    observation.add(R, "R");
    observation.add(T, "T");
    observation.commit();
    return 0;
  }
  DB db_;
  ObjectInstance object_instance_;
};

struct ObjectReader
{
  static void declare_params(tendrils& params)
  {
  }

  static void declare_io(const tendrils& params, const tendrils& inputs,
                         tendrils& outputs)
  {

  }
  void configure(tendrils& params)
  {
    db_.connect("object_db");
  }
  int process(const tendrils& inputs, tendrils& outputs)
  {
    return 0;
  }
  DB db_;
};

}

BOOST_PYTHON_MODULE(objcog_db)
{

}
;
