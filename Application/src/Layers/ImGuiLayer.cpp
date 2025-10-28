#include "ImGuiLayer.h"

#include "Events.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Core/EventBus.h"

#include "GLFW/glfw3.h"

ImGuiLayer::ImGuiLayer(EventBus& eventBus)
	: ILayer(eventBus)
{
}

void ImGuiLayer::OnInit()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
}

void ImGuiLayer::OnUpdate(float deltaTime)
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGui::Begin("Hello from ImGuiLayer");
	ImGui::Text("FrameRate: %d", static_cast<int>(ImGui::GetIO().Framerate));
	if (ImGui::Button("Toggle Update", ImVec2(-1.0f, 0.0f)))
	{
		ToggleUpdateEvent event;
		eventBus_.Publish(event);
	}
	if (ImGui::Button("Step", ImVec2(-1.0f, 0.0f)))
	{
		StepEvent event;
		event.DeltaTime = deltaTime;
		eventBus_.Publish(event);
	}
	ImGui::End();
}

void ImGuiLayer::OnRender(Renderer& renderer)
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiLayer::OnDestroy()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}