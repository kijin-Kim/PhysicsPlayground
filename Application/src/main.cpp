#include "Core/Application.h"
#include "Layers/ImGuiLayer.h"
#include "Layers/MainLayer.h"

int main()
{
	Application app( 1920, 1080);
	app.AddLayer<MainLayer>();
	app.AddLayer<ImGuiLayer>();
	app.Run();
}