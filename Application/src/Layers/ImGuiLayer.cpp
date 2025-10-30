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

	EventBus& eventBus = EventBus::GetInstance();

	PauseUpdateEvent pauseEvent;
	bShouldPauseUpdate_ = configFile.GetBool("Options", "Paused", false);
	pauseEvent.bPaused = bShouldPauseUpdate_;
	eventBus.Publish(pauseEvent);

	ChangeBroadPhaseAlgorithmEvent changeAlgorithmEvent;
	currentBroadPhaseType_ = BroadPhase::FromString(configFile.GetString("Options", "BroadPhaseAlgorithm", "Naive"));
	changeAlgorithmEvent.NewAlgorithm = currentBroadPhaseType_;
	eventBus.Publish(changeAlgorithmEvent);

	GridCellSizeChangedEvent cellSizeChangedEvent;
	currentGridCellSize_ = configFile.GetFloat("GridSettings", "GridCellSize", 0.0f);
	cellSizeChangedEvent.NewCellSize = currentGridCellSize_;
	eventBus.Publish(cellSizeChangedEvent);

	QuadTreeMaxObjectsPerNodeChangedEvent maxObjectsEvent;
	currentQuadTreeMaxObjectsPerNode_ = configFile.GetInt("QuadTreeSettings", "MaxObjectsPerNode", 2);
	maxObjectsEvent.NewMaxObjectsPerNode = currentQuadTreeMaxObjectsPerNode_;
	eventBus.Publish(maxObjectsEvent);

	QuadTreeMaxDepthChangedEvent maxDepthEvent;
	currentQuadTreeMaxDepth_ = configFile.GetInt("QuadTreeSettings", "MaxDepth", 6);
	maxDepthEvent.NewMaxDepth = currentQuadTreeMaxDepth_;
	eventBus.Publish(maxDepthEvent);
}

void ImGuiLayer::OnUpdate(float deltaTime)
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGui::Begin("Hello from ImGuiLayer");

	ImGui::SeparatorText("Options");
	ImGui::Text("FrameRate: %d (%.0f ms)", static_cast<int>(ImGui::GetIO().Framerate), 1000.0f / ImGui::GetIO().Framerate);

	EventBus& eventBus = EventBus::GetInstance();
	if (ImGui::Button("Pause/Resume", ImVec2(-1.0f, 0.0f)))
	{
		PauseUpdateEvent event;
		bShouldPauseUpdate_ = !bShouldPauseUpdate_;
		event.bPaused = bShouldPauseUpdate_;
		eventBus.Publish(event);
	}
	if (ImGui::Button("Step", ImVec2(-1.0f, 0.0f)))
	{
		StepEvent event;
		event.DeltaTime = deltaTime;
		eventBus.Publish(event);
	}

	std::array<std::string, static_cast<int>(BroadPhase::Type::NumTypes)> broadPhaseAlgorithms = {};
	for (int i = 0; i < broadPhaseAlgorithms.size(); ++i)
	{
		broadPhaseAlgorithms[i] = BroadPhase::ToString(static_cast<BroadPhase::Type>(i));
	}

	ImGui::SeparatorText("Broad Phase Settings");
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
				eventBus.Publish(event);
			}
			if (isSelected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}

	switch (currentBroadPhaseType_)
	{
	case BroadPhase::Type::Naive:
		break;
	case BroadPhase::Type::Grid:
	{
		static float gridCellSize = currentGridCellSize_;
		if (ImGui::SliderFloat("Grid Cell Size", &gridCellSize, 8.0f, 128.0f))
		{
			GridCellSizeChangedEvent event;
			currentGridCellSize_ = gridCellSize;
			event.NewCellSize = gridCellSize;
			eventBus.Publish(event);
		}
		break;
	}
	case BroadPhase::Type::Quadtree:
	{
		static int quadTreeMaxObjectsPerNode = currentQuadTreeMaxObjectsPerNode_;
		static int quadTreeMaxDepth = currentQuadTreeMaxDepth_;
		if (ImGui::SliderInt("QuadTree Max Objects Per Node", &quadTreeMaxObjectsPerNode, 1, 10))
		{
			currentQuadTreeMaxObjectsPerNode_ = quadTreeMaxObjectsPerNode;
			// You can publish an event here if needed
			QuadTreeMaxObjectsPerNodeChangedEvent event;
			event.NewMaxObjectsPerNode = quadTreeMaxObjectsPerNode;
			eventBus.Publish(event);
		}
		if (ImGui::SliderInt("QuadTree Max Depth", &quadTreeMaxDepth, 1, 10))
		{
			currentQuadTreeMaxDepth_ = quadTreeMaxDepth;
			QuadTreeMaxDepthChangedEvent event;
			event.NewMaxDepth = quadTreeMaxDepth;
			eventBus.Publish(event);
		}
		break;
	}
	case BroadPhase::Type::SAP:
		break;
	default:
		break;
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