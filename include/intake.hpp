#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(21);
inline pros::Motor intakeTop(-10);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();