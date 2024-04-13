#include "MyFlashLightPlugin.hh"
#include <thread>

using namespace gazebo;

MyFlashLightPlugin::MyFlashLightPlugin() {}

MyFlashLightPlugin::~MyFlashLightPlugin() {}

void MyFlashLightPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  FlashLightPlugin::Load(_model, _sdf);
  // Turn on the initial light
  std::thread myThread(&MyFlashLightPlugin::CycleLights, this);
  myThread.detach();
  // CycleLights();
  //  You can also set the initial timer here if needed
}

void MyFlashLightPlugin::TurnOnLight(int Index)
{
  // Turn on the light
  if (!this->TurnOn(lights[Index], "link"))
  {
    gzerr << "Failed to turn on light: " << lights[Index] << std::endl;
  }
}

void MyFlashLightPlugin::TurnOffLight(int Index)
{
  // Turn off the light
  if (!this->TurnOff(lights[Index], "link"))
  {
    gzerr << "Failed to turn off light: " << lights[Index] << std::endl;
  }
}

void MyFlashLightPlugin::CycleLights()
{
  while (true)
  {
    // Turn off the current light
    TurnOffLight(currentLight);

    // Move to the next light
    if (currentLight < 2)
    {
      currentLight = currentLight + 1;
    }
    else
    {
      currentLight = 0;
    }
    // Turn on the next light
    TurnOnLight(currentLight);
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
}


GZ_REGISTER_MODEL_PLUGIN(MyFlashLightPlugin)
