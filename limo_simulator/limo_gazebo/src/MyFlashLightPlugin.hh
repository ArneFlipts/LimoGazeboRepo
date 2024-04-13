#ifndef MY_FLASH_LIGHT_PLUGIN_HH
#define MY_FLASH_LIGHT_PLUGIN_HH

#include <gazebo-11/gazebo/gazebo.hh>
#include <gazebo-11/gazebo/physics/physics.hh>
#include <gazebo-11/gazebo/common/common.hh>
#include <gazebo-11/gazebo/common/Time.hh>
#include <gazebo-11/gazebo/plugins/FlashLightPlugin.hh>
#include <string>
#include <unistd.h>
#include <vector>
#include <thread>
#include <atomic>

namespace gazebo
{
  class MyFlashLightPlugin : public FlashLightPlugin
  {
  public:
    MyFlashLightPlugin();
    virtual ~MyFlashLightPlugin();

    // Overridden functions
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void onShutdown();

    void TurnOnLight(int index);
    void TurnOffLight(int index);
    void CycleLights();

  private:
    std::vector<std::string> lights = {"light_source1", "light_source2", "light_source3"};
    int currentLight = 0;
    std::thread myThread;
    std::atomic<bool> shouldRunThread{false};
  };
}

#endif // MY_FLASH_LIGHT_PLUGIN_HH