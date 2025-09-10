#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(21);
inline pros::Motor intakeTop(-10);


//int intakeLiftTracker = 0;

inline ez::Piston intakeLift('A');
inline ez::Piston parkLeft('E');
inline ez::Piston parkRight('F');

void IntakeLiftDrop(bool IntakeLiftState);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();
void IntakeLiftControl();
void IntakeParkControl();