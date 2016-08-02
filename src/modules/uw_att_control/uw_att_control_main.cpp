
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uw_att_control_main.cpp
 *
 * HippoCampus Underwater Attitude Controller. Feed Forward RC Input except roll. Balance roll.
 *
 * Based on rover steering control example by Lorenz Meier <lorenz@px4.io>
 *
 * @author Max Kirchhoff
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>


/* Prototypes */

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int uw_att_control_main(int argc, char *argv[]);

struct params {
    float roll_p;
    float roll_d;
};

struct param_handles {
    param_t roll_p;
    param_t roll_d;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Mainloop of daemon.
 */
int uw_att_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Control roll angle.
 *
 * Roll is controlled to zero everything else remains uncontrolled (only for manual flight).
 *
 */
void control_attitude(struct vehicle_attitude_s *att, struct actuator_controls_s *actuators,
                      struct manual_control_setpoint_s *manual_sp);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params pp;
static struct param_handles ph;

int parameters_init(struct param_handles *h)
{
    /* PD parameters */
    h->roll_p 	=	param_find("UW_ROLL_P");
    h->roll_d 	=	param_find("UW_ROLL_D");

    return OK;
}

int parameters_update(const struct param_handles *h, struct params *p)
{
    param_get(h->roll_p, &(p->roll_p));
    param_get(h->roll_d, &(p->roll_d));

    return OK;
}

void control_attitude( struct vehicle_attitude_s *att, struct actuator_controls_s *actuators,
                       struct manual_control_setpoint_s *manual_sp)
{
    /*
     * Control Group 0 (attitude):
     *
     *    0  -  roll   (-1..+1)
     *    1  -  pitch  (-1..+1)
     *    2  -  yaw    (-1..+1)
     *    3  -  thrust ( 0..+1) !needs to be adjusted for underwater use!
     *    4  -  flaps  (-1..+1)
     *    ...
     */

    /*
     * Calculate roll error and apply PD gain
     */

    float pcontrol = (0.0f - att->roll) * pp.roll_p;
    float dcontrol = (0.0f - att->rollspeed) * pp.roll_d;
    float pd_control = pcontrol + dcontrol;

    /* set actuator outputs*/
    actuators->control[0] = pd_control;
    actuators->control[1] = manual_sp->x;
    actuators->control[2] = manual_sp->y;
    actuators->control[3] = (manual_sp->z - 0.5f) * 2.0f; //z range: 0...+1 needs to be adjusted


    actuators->timestamp = hrt_absolute_time();
}

/* Main Thread */
int uw_att_control_thread_main(int argc, char *argv[])
{
    /* read arguments */
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        }
    }

    /* initialize parameters, first the handles, then the values */
    parameters_init(&ph);
    parameters_update(&ph, &pp);

    /*
     * Declare and safely initialize all structs to zero.
     *
     */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct manual_control_setpoint_s manual_sp;
    memset(&manual_sp, 0, sizeof(manual_sp));

   /* output structs - this is what is sent to the mixer */
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));


   /* publish actuator controls with zero values */
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }

    /*
     * Advertise that this controller will publish actuator
     * control values and the rate setpoint
     */
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

  /* subscribe to topics. */
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

    int param_sub = orb_subscribe(ORB_ID(parameter_update));

  /* Setup of loop */

    struct pollfd fds[2];

    fds[0].fd = att_sub;

    fds[0].events = POLLIN;

    fds[1].fd = param_sub;

    fds[1].events = POLLIN;


    while (!thread_should_exit) {

        /*
         * Wait for a sensor or param update, check for exit condition every 500 ms.
         * This means that the execution will block here without consuming any resources,
         * but will continue to execute the very moment a new attitude measurement or
         * a param update is published. So no latency in contrast to the polling
         * design pattern (do not confuse the poll() system call with polling).
         *
         * This design pattern makes the controller also agnostic of the attitude
         * update speed - it runs as fast as the attitude updates with minimal latency.
         */
        int ret = poll(fds, 2, 500);

        if (ret < 0) {
            /*
             * Poll error, this will not really happen in practice,
             * but its good design practice to make output an error message.
             */
            warnx("poll error");

        } else if (ret == 0) {
            /* no return value = nothing changed for 500 ms, ignore */
            warnx("nothing changed");
        } else {

            /* only update parameters if they changed */
            if (fds[1].revents & POLLIN) {
                /* read from param to clear updated flag (uORB API requirement) */
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), param_sub, &update);

                /* if a param update occured, re-read our parameters */
                parameters_update(&ph, &pp);
            }

            /* only run controller if attitude changed */
            if (fds[0].revents & POLLIN) {


                /* Check if there is a new position measurement or position setpoint */
                bool manual_sp_updated;
                orb_check(manual_sp_sub, &manual_sp_updated);

                if (manual_sp_updated)
                    /* get the RC (or otherwise user based) input */
                {
                    orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                }

                /* get a local copy of attitude */
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);


                /* control attitude */

                control_attitude(&att, &actuators, &manual_sp);

               /* sanity check and publish actuator outputs */
                if (isfinite(actuators.control[0]) &&
                    isfinite(actuators.control[1]) &&
                    isfinite(actuators.control[2]) &&
                    isfinite(actuators.control[3])) {
                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

                    if (verbose) {
                        warnx("published");
                    }
                }
            }
        }
    }

    warnx("exiting, stopping all motors.");
    thread_running = false;

    /* kill all outputs */
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }

    actuators.timestamp = hrt_absolute_time();

    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

    fflush(stdout);

    return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: uw_att_control {start|stop|status}\n\n");
    exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int uw_att_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("running");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        deamon_task = px4_task_spawn_cmd("uw_att_control",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 20,
                         2048,
                         uw_att_control_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        thread_running = true;
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");

        } else {
            warnx("not started");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}



