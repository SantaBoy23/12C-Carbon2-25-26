#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(21);
inline pros::Motor intakeTop(-10);

inline ez::Piston intakeLift('A');

inline bool IntakeLiftDropBool = false;

void IntakeLiftDrop(bool IntakeLiftState);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();
void IntakeLiftControl();