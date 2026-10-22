/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

/* CHIP project configuration (sdkconfig: CONFIG_CHIP_PROJECT_CONFIG). */

/* CASE sessions this device can set up at the same time (default 2). On boot it resumes the subscriptions of the
 * controllers (e.g. Apple Home and Home Assistant) and subscribes to the bound sensor: with 2 the third one failed
 * with CHIP_ERROR_NO_MEMORY, Apple Home then had no subscription until its max interval ran out ("No Response",
 * changed values jumped back). */
#define CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS 4
