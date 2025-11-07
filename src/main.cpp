#include "main.h"
#include "liblvgl/lvgl.h"



// =======================
// DEVICE DEFINITIONS
// =======================

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Subsystem ports
pros::Motor topIntake(3);
pros::Motor horizontalOdom(9);
pros::Motor trainRight1(11);
pros::Motor trainRight2(12);
pros::Motor trainRight3(13);
pros::Motor trainLeft1(14);
pros::Motor trainLeft2(15);
pros::Motor trainLeft3(16);
pros::Motor bottomIntake(21);

// =======================
// CHASSIS SETUP
// =======================

ez::Drive chassis(
    {-14, -15, -16},    // Left Chassis Ports
    {11, 12, 13},       // Right Chassis Ports
    17, 3.25, 450       // IMU Port, Wheel Diameter (in), Wheel RPM
);

ez::tracking_wheel horiz_tracker(9, 2, 3.38);  // Perpendicular tracker
ez::tracking_wheel vert_tracker(3, 2.2, 2.73); // Parallel tracker

// =======================
// CONNECTION CHECK FUNCTION
// =======================
bool checkConnections() {
  bool allGood = true;

  // --- Clear LCD ---
  for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
  pros::lcd::print(0, "Running device diagnostics...");

  // --- Motor Checks ---
  std::vector<pros::Motor*> motors = {
      &topIntake, &trainRight1, &trainRight2, &trainRight3,
      &trainLeft1, &trainLeft2, &trainLeft3, &bottomIntake
  };

  int line = 1;
  for (auto m : motors) {
    if (!m->is_installed()) {
      std::cout << "⚠️ Motor missing on port " << m->get_port() << std::endl;
      pros::lcd::print(line++, "⚠ Motor %d missing!", m->get_port());
      allGood = false;
      pros::delay(500);
    }
  }

  // --- Results ---
  if (allGood) {
    std::cout << "✅ All devices connected!" << std::endl;
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::lcd::print(0, "✅ All devices OK!");
  } else {
    pros::lcd::print(line + 1, "❌ Check wiring before run!");
  }

  return allGood;
}


// =======================
// LVGL IMAGE DISPLAY + ROTATION
// =======================

// extern lv_img_dsc_t screen_logo;

// LV_IMG_DECLARE(screen_logo); // Replace with your image variable name

// void rotate_logo_task(void *param) {
//   lv_obj_t *img = (lv_obj_t *)param;
//   int angle = 0;

//   while (true) {
//     lv_img_set_angle(img, angle * 10); // LVGL uses 0.1 degree units
//     angle = (angle + 2) % 360; // smooth spin
//     pros::delay(50);
//   }
// }

// void setup_rotating_logo() {
//   // Create the image on the screen
//   lv_obj_t* img = lv_img_create(lv_scr_act());
//   lv_img_set_src(img, &screen_logo); // your LVGL image symbol
//   lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

//   // optional: set rotation pivot to image center
//   //lv_img_set_pivot(img, 64, 64); // adjust based on image size

//   // Start rotation task
//   //pros::Task rotateTask(rotate_logo_task, img);
// }


// =======================
// INITIALIZE
// =======================
void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  pros::lcd::initialize();  // Required for screen output
  for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
  pros::lcd::print(0, "Checking devices...");
  std::cout << "Running device diagnostics..." << std::endl;

  // bool good = checkConnections();
  // if (!good) {
  //   std::cout << "❌ Connection check failed! Halting init." << std::endl;
  //   pros::lcd::print(1, "❌ Connection check failed!");
  //   pros::lcd::print(2, "Fix wires & reboot.");
  //   while (true) {  // Stop robot until reboot
  //     pros::delay(100);
  //   }
  // }

  // --- Continue with EZ-Template setup ---
  chassis.odom_tracker_front_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  // chassis.opcontrol_curve_default_set(0.0, 0.0);
  default_constants();

  // --- AUTON SELECTOR ---
  ez::as::auton_selector.autons_add({
      {"Red right side elims centered block rush", elims_rush_right_red},
      {"Blue right side elims centered block rush", elims_rush_right_blue},
      {"Red right 7 block auto", red_right_7_auto},
      {"Blue right 7 block auto", blue_right_7_auto},
      {"Red Left auto", red_left_auto},
      {"RED Right Side Solo AWP", red_right_solo},
      {"RED Right Side auto", red_right},
      {"BLUE Right Side auto", blue_right},
      {"Skills Auto", skills_auto},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  chassis.initialize();
  ez::as::initialize();

  // ===============================
  // "Check" Button (Top Left)
  // ===============================
  lv_obj_t* check_btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(check_btn, 70, 40);
  lv_obj_align(check_btn, LV_ALIGN_TOP_LEFT, 10, 10);

  lv_obj_t* check_label = lv_label_create(check_btn);
  lv_label_set_text(check_label, "Check");
  lv_obj_center(check_label);

  // Static pointer to the running task
  static pros::Task* checkTask = nullptr;

  // Event: Open Connection Status Page
  lv_obj_add_event_cb(check_btn, [](lv_event_t* e) {
      lv_obj_clean(lv_scr_act()); // clear auton selector

      // --- Title ---
      lv_obj_t* title = lv_label_create(lv_scr_act());
      lv_label_set_text(title, "🔌 Device Connection Check");
      lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

      // --- Scrollable Container ---
      lv_obj_t* container = lv_obj_create(lv_scr_act());
      lv_obj_set_size(container, 460, 180);
      lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 35);
      lv_obj_set_scroll_dir(container, LV_DIR_VER);
      lv_obj_set_style_pad_row(container, 4, 0);
      lv_obj_set_style_pad_all(container, 6, 0);

      // --- Back Button ---
      lv_obj_t* back_btn = lv_btn_create(lv_scr_act());
      lv_obj_set_size(back_btn, 70, 40);
      lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -5);
      lv_obj_t* back_label = lv_label_create(back_btn);
      lv_label_set_text(back_label, "Back");
      lv_obj_center(back_label);

      // --- Create Live-Updating Check Task ---
      checkTask = new pros::Task([container]() {
          struct Device {
              std::string name;
              pros::Motor* motor;
              lv_obj_t* labelStatus;
          };

          std::vector<Device> devices;
          std::vector<std::pair<std::string, pros::Motor*>> motorDefs = {
              {"Top Intake (3)",      &topIntake},
              {"Horizontal Odom (9)", &horizontalOdom},
              {"Train Right 1 (11)",  &trainRight1},
              {"Train Right 2 (12)",  &trainRight2},
              {"Train Right 3 (13)",  &trainRight3},
              {"Train Left 1 (14)",   &trainLeft1},
              {"Train Left 2 (15)",   &trainLeft2},
              {"Train Left 3 (16)",   &trainLeft3},
              {"Bottom Intake (21)",  &bottomIntake}
          };

          for (auto& m : motorDefs) {
              lv_obj_t* line = lv_obj_create(container);
              lv_obj_set_size(line, 420, 25);
              lv_obj_set_flex_flow(line, LV_FLEX_FLOW_ROW);
              lv_obj_set_style_pad_all(line, 2, 0);

              lv_obj_t* labelName = lv_label_create(line);
              lv_label_set_text_fmt(labelName, "%s", m.first.c_str());
              lv_obj_set_width(labelName, 220);

              lv_obj_t* labelStatus = lv_label_create(line);
              lv_label_set_text(labelStatus, "...");
              lv_obj_set_style_text_color(labelStatus, lv_color_hex(0xAAAAAA), 0);

              devices.push_back({m.first, m.second, labelStatus});
          }

          while (true) {
              for (auto& d : devices) {
                  bool connected = d.motor->is_installed();
                  lv_label_set_text(d.labelStatus, connected ? "OK" : "Missing");
                  lv_obj_set_style_text_color(
                      d.labelStatus,
                      connected ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000),
                      0
                  );
              }
              pros::delay(2000);
          }
      });

      // --- Back Button Functionality ---
      lv_obj_add_event_cb(back_btn, [](lv_event_t* e2) {
          if (checkTask) {
              checkTask->remove();
              delete checkTask;
              checkTask = nullptr;
          }

          lv_obj_clean(lv_scr_act());
          ez::as::initialize(); // return to auton selector
      }, LV_EVENT_CLICKED, NULL);

  }, LV_EVENT_CLICKED, NULL);




  // ===============================
  // "Logo" Button (Top Right)
  // ===============================
  
  lv_obj_t* caffeine_btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(caffeine_btn, 60, 40); // button size
  lv_obj_align(caffeine_btn, LV_ALIGN_TOP_RIGHT, -10, 10); // top-right corner

  lv_obj_t* label = lv_label_create(caffeine_btn);
  lv_label_set_text(label, "Logo");
  lv_obj_center(label);

  // When pressed -> go to logo page
  lv_obj_add_event_cb(caffeine_btn, [](lv_event_t* e) {
    // Clear the current screen
    lv_obj_clean(lv_scr_act());

    // Create and show logo image
    extern const lv_img_dsc_t screen_logo; // your image descriptor
    lv_obj_t* img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &screen_logo);
    lv_obj_center(img);

    // Rotate logo
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, img);
    lv_anim_set_values(&a, 0, 3600); // 3600 tenths of a degree (360°)
    lv_anim_set_time(&a, 8000); // 8 seconds per full spin
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&a, [](void* obj, int32_t v) {
        lv_img_set_angle((lv_obj_t*)obj, v);
    });
    lv_anim_start(&a);

    // Add back button when in log program
    lv_obj_t* back_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(back_btn, 60, 40);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);

    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        // Go back to auton selector
        ez::as::initialize();
    }, LV_EVENT_CLICKED, NULL);
  }, LV_EVENT_CLICKED, NULL);


  // pros::lcd::print(3, "IMU Calibrated: %s", chassis.drive_imu_calibrated() ? "Yes" : "No");

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  
}


// =======================
// THE REST OF THE CODE
// =======================

void disabled() {}
void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()), 1);
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    } else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle(), chassis.pid_tuner_full_enable(true);

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    chassis.pid_tuner_iterate();
  } else if (chassis.pid_tuner_enabled()) {
    chassis.pid_tuner_disable();
  }
}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();
    chassis.opcontrol_tank();
    IntakeControl();
    CenterDescoreControl();
    IntakeLiftControl();
    MatchLoadControl();
    AntennaControl();
    IntakeParkControl();

    pros::delay(ez::util::DELAY_TIME);
  }
}
