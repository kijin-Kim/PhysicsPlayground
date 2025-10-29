#include "ImGuiLayer.h"

#include <array>
#include <string>

#include "Events.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Core/EventBus.h"

#include "GLFW/glfw3.h"
#include <Windows.h>
#include <filesystem>

#include "Core/IniParser.h"

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

	std::filesystem::path configPath = CONFIG_PATH;
	configPath /= "config.ini";
	IniFile configFile = ParseIniFile(configPath);

	PauseUpdateEvent pauseEvent;
	bShouldPauseUpdate_ = configFile.GetBool("Options", "Paused", false);
	pauseEvent.bPaused = bShouldPauseUpdate_;
	eventBus_.Publish(pauseEvent);
	ChangeBroadPhaseAlgorithmEvent changeAlgorithmEvent;
	currentBroadPhaseType_ = BroadPhase::FromString(configFile.GetString("Options", "BroadPhaseAlgorithm", "Naive"));
	changeAlgorithmEvent.NewAlgorithm = currentBroadPhaseType_;
	eventBus_.Publish(changeAlgorithmEvent);
	GridCellSizeChangedEvent cellSizeChangedEvent;
	currentGridCellSize_ = configFile.GetFloat("Options", "GridCellSize", 0.0f);
	cellSizeChangedEvent.NewCellSize = currentGridCellSize_;
	eventBus_.Publish(cellSizeChangedEvent);
}

void ImGuiLayer::OnUpdate(float deltaTime)
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGui::Begin("Hello from ImGuiLayer");
	ImGui::Text("FrameRate: %d", static_cast<int>(ImGui::GetIO().Framerate));
	if (ImGui::Button("Pause/Resume", ImVec2(-1.0f, 0.0f)))
	{
		PauseUpdateEvent event;
		bShouldPauseUpdate_ = !bShouldPauseUpdate_;
		event.bPaused = bShouldPauseUpdate_;
		eventBus_.Publish(event);
	}
	if (ImGui::Button("Step", ImVec2(-1.0f, 0.0f)))
	{
		StepEvent event;
		event.DeltaTime = deltaTime;
		eventBus_.Publish(event);
	}

	std::array<std::string, 2> broadPhaseAlgorithms = {};
	for (int i = 0; i < broadPhaseAlgorithms.size(); ++i)
	{
		broadPhaseAlgorithms[i] = BroadPhase::ToString(static_cast<BroadPhase::Type>(i));
	}

	static int currentBroadPhaseAlgorithm = static_cast<int>(currentBroadPhaseType_);
	if (ImGui::BeginCombo("Broad Phase Algorithm", BroadPhase::ToString(currentBroadPhaseType_)))
	{
		for (int n = 0; n < broadPhaseAlgorithms.size(); n++)
		{
			bool isSelected = (currentBroadPhaseAlgorithm == n);
			if (ImGui::Selectable(broadPhaseAlgorithms[n].c_str(), isSelected))
			{
				currentBroadPhaseAlgorithm = n;
				ChangeBroadPhaseAlgorithmEvent event;
				currentBroadPhaseType_ = static_cast<BroadPhase::Type>(n);
				event.NewAlgorithm = static_cast<BroadPhase::Type>(n);
				eventBus_.Publish(event);
			}
			if (isSelected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}

	static float cellSize = 32.0f;
	if (ImGui::SliderFloat("Grid Cell Size", &cellSize, 8.0f, 128.0f))
	{
		GridCellSizeChangedEvent event;
		event.NewCellSize = cellSize;
		eventBus_.Publish(event);
	}

	ImGui::ShowDemoWindow();

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