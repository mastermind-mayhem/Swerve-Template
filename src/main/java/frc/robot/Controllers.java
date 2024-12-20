package frc.robot;

public final class Controllers {

  // Create options for what gears the drivetrain can use.
  public enum Gears {
    LOW,
    HIGH
  }

  // Default Driver Controller configuraton (Xbox Pro)
  public static int DRIVER_ONE_PORT = 0;
  public static int DRIVER_SEC_PORT = 0;
  public static int DRIVER_TURN = 4;
  public static int DRIVER_SPEED = 1;
  public static double LOWEREDSPEED = 0.55;
  public static boolean DRIVER_W_BUTTONS = false;
  public static Controllers.Gears CURRENT_SPEEDLIMIT = Gears.HIGH;
  public static boolean arcadedriver = true;

  // Operator Controller Configuration (Logitech Dual Action)
  public static int OPER_PORT = 3;

  // XBOX PRO
  // Joysticks
  public static int xbox_lx = 0;
  public static int xbox_ly = 1;
  public static int xbox_lt = 2;
  public static int xbox_rt = 3;
  public static int xbox_rx = 4;
  public static int xbox_ry = 5;

  // Buttons
  public static int xbox_a = 1;
  public static int xbox_b = 2;
  public static int xbox_x = 3;
  public static int xbox_y = 4;
  public static int xbox_lb = 5;
  public static int xbox_rb = 6;
  public static int xbox_share = 7;
  public static int xbox_options = 8;
  public static int xbox_lbutton = 9;
  public static int xbox_rbutton = 10;

  // Logitech Dual Action
  // Joysticks
  public static int logi_lx = 0;
  public static int logi_ly = 1;
  public static int logi_rx = 2;
  public static int logi_ry = 3;

  // Buttons
  public static int logi_x = 1;
  public static int logi_a = 2;
  public static int logi_b = 3;
  public static int logi_y = 4;
  public static int logi_lb = 5;
  public static int logi_rb = 6;
  public static int logi_lt = 7;
  public static int logi_rt = 8;
  public static int logi_back = 9;
  public static int logi_start = 10;
  public static int logi_lbutton = 11;
  public static int logi_rbutton = 12;

  // PS4 Controller
  // Joysticks
  public static int ps4_lx = 0;
  public static int ps4_ly = 1;
  public static int ps4_LT = 2;
  public static int ps4_RT = 3;
  public static int ps4_rx = 4;
  public static int ps4_ry = 5;

  // Buttons
  public static int ps4_X = 1;
  public static int ps4_O = 2;
  public static int ps4_square = 3;
  public static int ps4_triangle = 4;
  public static int ps4_LB = 5;
  public static int ps4_RB = 6;
  public static int ps4_share = 7;
  public static int ps4_options = 8;
  public static int ps4_LJoystick = 9;
  public static int ps4_RJoystick = 10;
}
