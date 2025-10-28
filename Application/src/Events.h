#pragma once
#include "Core/EventBus.h"


struct ToggleUpdateEvent : IEvent
{
};

struct StepEvent : IEvent
{
	float DeltaTime;
};
