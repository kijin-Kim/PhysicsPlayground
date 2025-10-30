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
		Grid,
		Quadtree,
		SAP,
		NumTypes
	};

	inline const char* ToString(Type e)
	{
		switch (e)
		{
		case Type::Naive:
			return "Naive";
		case Type::Grid:
			return "Grid";
		case Type::Quadtree:
			return "Quadtree";
		case Type::SAP:
			return "SAP";
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

		if (str == "Quadtree")
		{
			return Type::Quadtree;
		}

		if (str == "SAP")
		{
			return Type::SAP;
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

struct QuadTreeMaxDepthChangedEvent : IEvent
{
	int NewMaxDepth;
};

struct QuadTreeMaxObjectsPerNodeChangedEvent : IEvent
{
	int NewMaxObjectsPerNode;
};