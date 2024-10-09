
"use strict";

let get_angle_act = require('./get_angle_act.js')
let get_status = require('./get_status.js')
let set_speed = require('./set_speed.js')
let get_error = require('./get_error.js')
let set_angle_flexible = require('./set_angle_flexible.js')
let set_force = require('./set_force.js')
let joint_set_zero = require('./joint_set_zero.js')
let set_clear_error = require('./set_clear_error.js')
let set_angle = require('./set_angle.js')
let get_force_act = require('./get_force_act.js')

module.exports = {
  get_angle_act: get_angle_act,
  get_status: get_status,
  set_speed: set_speed,
  get_error: get_error,
  set_angle_flexible: set_angle_flexible,
  set_force: set_force,
  joint_set_zero: joint_set_zero,
  set_clear_error: set_clear_error,
  set_angle: set_angle,
  get_force_act: get_force_act,
};
