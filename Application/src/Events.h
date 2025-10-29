#pragma once
#include <string>

#include "Core/EventBus.h"


struct PauseUpdateEvent : IEvent
{
	bool bPaused;
};

struct StepEvent : IEvent
{
	float DeltaTime;
};


namespace BroadPhase
{
	enum class Type
	{
		Naive = 0,
		Grid
	};

	inline const char* ToString(Type e)
	{
		switch (e)
		{
		case Type::Naive:
			return "Naive";
		case Type::Grid:
			return "Grid";
		default:
			return "unknown";
		}
	}

	inline Type FromString(const std::string& str)
	{
		if (str == "Naive")
		{
			return Type::Naive;
		}

		if (str == "Grid")
		{
			return Type::Grid;
		}
	}
}


struct ChangeBroadPhaseAlgorithmEvent : IEvent
{
	BroadPhase::Type NewAlgorithm;
};

struct GridCellSizeChangedEvent : IEvent
{
	float NewCellSize;
};