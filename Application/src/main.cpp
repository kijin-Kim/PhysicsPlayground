#include "Core/Application.h"
#include "Layers/ImGuiLayer.h"
#include "Layers/MainLayer.h"

#include "common/TracySystem.hpp"

int main()
{
	tracy::SetThreadName("Main Thread");
	Application app(1920, 1080);
	app.AddLayer<MainLayer>();
	app.AddLayer<ImGuiLayer>();
	app.Run();
}