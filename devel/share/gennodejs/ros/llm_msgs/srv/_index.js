
"use strict";

let set_speed = require('./set_speed.js')
let joint_set_zero = require('./joint_set_zero.js')
let set_angle_flexible = require('./set_angle_flexible.js')
let get_force_act = require('./get_force_act.js')
let set_force = require('./set_force.js')
let set_clear_error = require('./set_clear_error.js')
let get_status = require('./get_status.js')
let get_angle_act = require('./get_angle_act.js')
let set_angle = require('./set_angle.js')
let get_error = require('./get_error.js')

module.exports = {
  set_speed: set_speed,
  joint_set_zero: joint_set_zero,
  set_angle_flexible: set_angle_flexible,
  get_force_act: get_force_act,
  set_force: set_force,
  set_clear_error: set_clear_error,
  get_status: get_status,
  get_angle_act: get_angle_act,
  set_angle: set_angle,
  get_error: get_error,
};
