/*
 * E-puck maze controller — wersja finalna
 * Reguła lewej ręki, kalibracja, komunikacja leader/follower, detekcja wyjścia.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define TIME_STEP         64
#define MAX_SPEED         6.28
#define TURN_SPEED        (MAX_SPEED * 0.5)
#define FWD_SPEED         (MAX_SPEED * 0.8)

#define CALIBRATION_STEPS 20
#define DELTA_SIDE        3.0
#define DELTA_FRONT       6.0

#define TURN_STEPS        10
#define FORWARD_STEPS     12

#define GREEN_THR         50
#define COMM_CHANNEL      1

/* Stuck: jeśli robot nie przesunął się o STUCK_DIST przez STUCK_STEPS kroków */
#define STUCK_STEPS       200
#define STUCK_DIST        0.001

/* Obrót awaryjny gdy stuck: dwa razy TURN_STEPS = ~180° */
#define UTURN_STEPS       20

#define PS_FR  0
#define PS_RF  1
#define PS_R   2
#define PS_RB  3
#define PS_B   4
#define PS_LB  5
#define PS_L   6
#define PS_FL  7

typedef enum {
  NAV_FOLLOW,
  NAV_TURN_RIGHT,
  NAV_TURN_LEFT,
  NAV_FORWARD_AFTER_LEFT,
  NAV_UTURN,
  NAV_DONE
} NavState;

typedef enum { ROLE_UNKNOWN, ROLE_LEADER, ROLE_FOLLOWER } Role;

static int cmp_double(const void *a, const void *b) {
  double da = *(const double *)a, db = *(const double *)b;
  return (da > db) - (da < db);
}

static double green_fraction(WbDeviceTag camera) {
  int w = wb_camera_get_width(camera);
  int h = wb_camera_get_height(camera);
  int total = w * h;
  if (total == 0) return 0.0;
  const unsigned char *img = wb_camera_get_image(camera);
  int count = 0;
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      int g = wb_camera_image_get_green(img, w, x, y);
      int r = wb_camera_image_get_red(img, w, x, y);
      int b = wb_camera_image_get_blue(img, w, x, y);
      if (g > r + GREEN_THR && g > b + GREEN_THR) count++;
    }
  return (double)count / total;
}

int main(void) {
  wb_robot_init();

  const char *name = wb_robot_get_name();
  int my_id = (strcmp(name, "e-puck") == 0) ? 1 : 2;
  srand((unsigned int)(time(NULL) + my_id * 1000));

  /* Log do pliku */
  char logpath[128];
  snprintf(logpath, sizeof(logpath),
           "C:/Users/Administrator/Desktop/labirynt/%s.log", name);
  freopen(logpath, "w", stdout);
  setvbuf(stdout, NULL, _IOLBF, 0);

  WbDeviceTag lm = wb_robot_get_device("left wheel motor");
  WbDeviceTag rm = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(lm, INFINITY);
  wb_motor_set_position(rm, INFINITY);
  wb_motor_set_velocity(lm, 0.0);
  wb_motor_set_velocity(rm, 0.0);

  WbDeviceTag ps[8];
  for (int i = 0; i < 8; i++) {
    char buf[8];
    snprintf(buf, sizeof(buf), "ps%d", i);
    ps[i] = wb_robot_get_device(buf);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  wb_emitter_set_channel(emitter, COMM_CHANNEL);
  wb_receiver_set_channel(receiver, COMM_CHANNEL);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  /* --- Kalibracja --- */
  double cal[CALIBRATION_STEPS * 8];
  int cal_n = 0;
  for (int i = 0; i < CALIBRATION_STEPS; i++) {
    wb_robot_step(TIME_STEP);
    for (int j = 0; j < 8; j++)
      cal[cal_n++] = wb_distance_sensor_get_value(ps[j]);
  }
  qsort(cal, cal_n, sizeof(double), cmp_double);
  double baseline  = cal[cal_n / 2];
  double SIDE_THR  = baseline + DELTA_SIDE;
  double FRONT_THR = baseline + DELTA_FRONT;
  printf("[%s] kalibracja: baseline=%.1f SIDE=%.1f FRONT=%.1f\n",
         name, baseline, SIDE_THR, FRONT_THR);

  /* --- Stan roli --- */
  Role my_role = ROLE_UNKNOWN;
  int role_decided = 0;
  int id_sent = 0;
  double other_x = -1e9, other_y = -1e9;
  int other_age = 99999;

  /* --- Stan nawigacji --- */
  NavState nav = NAV_FOLLOW;
  int tc = 0, fc = 0, step = 0;

  /* --- Stuck detection --- */
  double last_x = -1e9, last_y = -1e9;
  int stuck_cnt = 0;

  printf("[%s] SIDE_THR=%.1f FRONT_THR=%.1f\n", name, SIDE_THR, FRONT_THR);

  while (wb_robot_step(TIME_STEP) != -1) {
    double pv[8];
    for (int i = 0; i < 8; i++)
      pv[i] = wb_distance_sensor_get_value(ps[i]);
    const double *pos = wb_gps_get_values(gps);
    step++;
    other_age++;

    /* --- Log co 50 kroków --- */
    if (step % 50 == 1)
      printf("[%s] step=%d nav=%d role=%d pos=(%.2f,%.2f) stuck=%d\n",
             name, step, (int)nav, (int)my_role,
             pos[0], pos[1], stuck_cnt);

    /* --- Wykrywanie wyjścia --- */
    if (nav != NAV_DONE && step % 5 == 0) {
      double gf = green_fraction(camera);
      if (gf > 0.30) {
        printf("[%s] WYJSCIE! gf=%.2f pos=(%.2f,%.2f)\n",
               name, gf, pos[0], pos[1]);
        wb_motor_set_velocity(lm, 0.0);
        wb_motor_set_velocity(rm, 0.0);
        char msg[64];
        snprintf(msg, sizeof(msg), "EXIT:%d:%.3f:%.3f", my_id, pos[0], pos[1]);
        wb_emitter_send(emitter, msg, (int)strlen(msg) + 1);
        nav = NAV_DONE;
        continue;
      } else if (gf > 0.05) {
        wb_motor_set_velocity(lm, FWD_SPEED);
        wb_motor_set_velocity(rm, FWD_SPEED);
        continue;
      }
    }

    if (nav == NAV_DONE) {
      wb_motor_set_velocity(lm, 0.0);
      wb_motor_set_velocity(rm, 0.0);
      continue;
    }

    /* --- Komunikacja: odbiór --- */
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *msg = (const char *)wb_receiver_get_data(receiver);
      if (strncmp(msg, "ID:", 3) == 0 && !role_decided) {
        int rid;
        if (sscanf(msg + 3, "%d", &rid) == 1 && rid != my_id) {
          my_role = (my_id < rid) ? ROLE_LEADER : ROLE_FOLLOWER;
          role_decided = 1;
          printf("[%s] -> %s (kolega ID=%d)\n",
                 name, my_role == ROLE_LEADER ? "LEADER" : "FOLLOWER", rid);
        }
      } else if (strncmp(msg, "POS:", 4) == 0) {
        int rid; double rx, ry;
        if (sscanf(msg + 4, "%d:%lf:%lf", &rid, &rx, &ry) == 3 && rid != my_id) {
          other_x = rx; other_y = ry; other_age = 0;
        }
      } else if (strncmp(msg, "EXIT:", 5) == 0) {
        int rid;
        if (sscanf(msg + 5, "%d", &rid) == 1 && rid != my_id) {
          printf("[%s] kolega znalazl wyjscie.\n", name);
          nav = NAV_DONE;
        }
      }
      wb_receiver_next_packet(receiver);
    }

    /* Wysyłanie ID */
    if (!role_decided && id_sent < 10 && step % 5 == 1) {
      char msg[32];
      snprintf(msg, sizeof(msg), "ID:%d", my_id);
      wb_emitter_send(emitter, msg, (int)strlen(msg) + 1);
      id_sent++;
    }
    /* Timeout solo */
    if (!role_decided && step > 60) {
      my_role = ROLE_LEADER;
      role_decided = 1;
      printf("[%s] solo -> LEADER\n", name);
    }

    /* Broadcast pozycji co 20 kroków */
    if (role_decided && step % 20 == 0) {
      char msg[64];
      snprintf(msg, sizeof(msg), "POS:%d:%.3f:%.3f", my_id, pos[0], pos[1]);
      wb_emitter_send(emitter, msg, (int)strlen(msg) + 1);
    }

    /* --- Ochrona granic: zawróć jeśli robot wychodzi poza labirynt --- */
    if (nav != NAV_UTURN && nav != NAV_DONE) {
      if (pos[1] < -0.05 || pos[0] < -0.05 || pos[0] > 2.05) {
        printf("[%s] GRANICA pos=(%.2f,%.2f) -> zawracam\n", name, pos[0], pos[1]);
        nav = NAV_UTURN;
        tc = 0;
        stuck_cnt = 0;
      }
    }

    /* --- Stuck detection --- */
    if (nav != NAV_UTURN) {
      if (last_x > -1e8) {
        double dx = pos[0] - last_x, dy = pos[1] - last_y;
        if (sqrt(dx*dx + dy*dy) < STUCK_DIST)
          stuck_cnt++;
        else
          stuck_cnt = 0;
        if (stuck_cnt > STUCK_STEPS) {
          printf("[%s] STUCK pos=(%.2f,%.2f) -> zawracam\n", name, pos[0], pos[1]);
          nav = NAV_UTURN;
          tc = 0;
          stuck_cnt = 0;
        }
      }
      last_x = pos[0]; last_y = pos[1];
    }

    /* --- Follower: zwalniaj gdy blisko lidera --- */
    if (my_role == ROLE_FOLLOWER && other_age < 30 && nav != NAV_UTURN) {
      double dx = other_x - pos[0];
      double dy = other_y - pos[1];
      if (sqrt(dx*dx + dy*dy) < 0.12) {
        wb_motor_set_velocity(lm, 0.0);
        wb_motor_set_velocity(rm, 0.0);
        continue;
      }
    }

    /* --- Czujniki ściany --- */
    int w_front = (pv[PS_FR] > FRONT_THR || pv[PS_FL] > FRONT_THR);
    int w_left  =  pv[PS_L]  > SIDE_THR;

    /* --- Automat stanów --- */
    if (nav == NAV_UTURN) {
      wb_motor_set_velocity(lm,  TURN_SPEED);
      wb_motor_set_velocity(rm, -TURN_SPEED);
      if (++tc >= UTURN_STEPS) { nav = NAV_FOLLOW; tc = 0; }
      continue;
    }

    if (nav == NAV_TURN_RIGHT) {
      wb_motor_set_velocity(lm,  TURN_SPEED);
      wb_motor_set_velocity(rm, -TURN_SPEED);
      if (++tc >= TURN_STEPS) { nav = NAV_FOLLOW; tc = 0; }
      continue;
    }

    if (nav == NAV_TURN_LEFT) {
      wb_motor_set_velocity(lm, -TURN_SPEED);
      wb_motor_set_velocity(rm,  TURN_SPEED);
      if (++tc >= TURN_STEPS) { nav = NAV_FORWARD_AFTER_LEFT; tc = 0; fc = 0; }
      continue;
    }

    if (nav == NAV_FORWARD_AFTER_LEFT) {
      wb_motor_set_velocity(lm, FWD_SPEED);
      wb_motor_set_velocity(rm, FWD_SPEED);
      if (++fc >= FORWARD_STEPS || w_front) { nav = NAV_FOLLOW; fc = 0; }
      continue;
    }

    /* NAV_FOLLOW — reguła lewej ręki */
    if (!w_left) {
      nav = NAV_TURN_LEFT; tc = 0;
    } else if (!w_front) {
      wb_motor_set_velocity(lm, FWD_SPEED);
      wb_motor_set_velocity(rm, FWD_SPEED);
    } else {
      nav = NAV_TURN_RIGHT; tc = 0;
    }
  }

  wb_motor_set_velocity(lm, 0.0);
  wb_motor_set_velocity(rm, 0.0);
  wb_robot_cleanup();
  return 0;
}
