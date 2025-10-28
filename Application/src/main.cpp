#include "Core/Application.h"
#include "Layers/ImGuiLayer.h"
#include "Layers/MainLayer.h"

int main()
{
	Application app( 800, 600);
	app.AddLayer<MainLayer>();
	app.AddLayer<ImGuiLayer>();
	app.Run();
}