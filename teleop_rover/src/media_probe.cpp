#include <gst/gst.h>

#include <initializer_list>
#include <iostream>
#include <string>

namespace
{
std::string first_factory(std::initializer_list<const char *> candidates)
{
  for (const char * name : candidates) {
    GstElementFactory * factory = gst_element_factory_find(name);
    if (factory != nullptr) {
      gst_object_unref(factory);
      return name;
    }
  }
  return {};
}

void print_codec(const char * codec, std::initializer_list<const char *> candidates)
{
  const auto factory = first_factory(candidates);
  std::cout << codec << "=" << (factory.empty() ? "unavailable" : factory) << '\n';
}
}

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  print_codec("AV1", {"nvav1enc", "vaav1enc", "svtav1enc", "av1enc"});
  print_codec("VP9", {"vavp9enc", "vp9enc"});
  print_codec("H264", {"nvh264enc", "vah264enc", "v4l2h264enc", "x264enc"});
  return 0;
}
