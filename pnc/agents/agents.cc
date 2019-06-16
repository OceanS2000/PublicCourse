// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/agents/agents.h"
#include "pnc/agents/sao_agent/sao_agent.h"
#include "pnc/agents/sample/sample_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::sao_agent::SaoVehicleAgent> registrar("sao_agent");
