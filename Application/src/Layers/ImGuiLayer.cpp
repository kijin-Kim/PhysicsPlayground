#include "ImGuiLayer.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "GLFW/glfw3.h"

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
	ImGui::Text("This is ImGui integrated as a layer!");
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