#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(1);
inline pros::Motor intakeTop(-7);


//int intakeLiftTracker = 0;

inline ez::Piston intakeLift('A');

void IntakeLiftDrop(bool IntakeLiftState);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();
void IntakeLiftControl();
void IntakeParkControl();
void FullIntakeMove();

void TestIntake();