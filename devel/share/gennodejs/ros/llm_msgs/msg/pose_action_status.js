// Auto-generated. Do not edit!

// (in-package llm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class pose_action_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.resp_frame_id = null;
      this.hand_move_success = null;
      this.head_move_success = null;
      this.gripper_move_success = null;
      this.waist_move_success = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('resp_frame_id')) {
        this.resp_frame_id = initObj.resp_frame_id
      }
      else {
        this.resp_frame_id = 0;
      }
      if (initObj.hasOwnProperty('hand_move_success')) {
        this.hand_move_success = initObj.hand_move_success
      }
      else {
        this.hand_move_success = 0;
      }
      if (initObj.hasOwnProperty('head_move_success')) {
        this.head_move_success = initObj.head_move_success
      }
      else {
        this.head_move_success = 0;
      }
      if (initObj.hasOwnProperty('gripper_move_success')) {
        this.gripper_move_success = initObj.gripper_move_success
      }
      else {
        this.gripper_move_success = 0;
      }
      if (initObj.hasOwnProperty('waist_move_success')) {
        this.waist_move_success = initObj.waist_move_success
      }
      else {
        this.waist_move_success = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pose_action_status
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [resp_frame_id]
    bufferOffset = _serializer.uint32(obj.resp_frame_id, buffer, bufferOffset);
    // Serialize message field [hand_move_success]
    bufferOffset = _serializer.uint8(obj.hand_move_success, buffer, bufferOffset);
    // Serialize message field [head_move_success]
    bufferOffset = _serializer.uint8(obj.head_move_success, buffer, bufferOffset);
    // Serialize message field [gripper_move_success]
    bufferOffset = _serializer.uint8(obj.gripper_move_success, buffer, bufferOffset);
    // Serialize message field [waist_move_success]
    bufferOffset = _serializer.uint8(obj.waist_move_success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pose_action_status
    let len;
    let data = new pose_action_status(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [resp_frame_id]
    data.resp_frame_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [hand_move_success]
    data.hand_move_success = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [head_move_success]
    data.head_move_success = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gripper_move_success]
    data.gripper_move_success = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [waist_move_success]
    data.waist_move_success = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'llm_msgs/pose_action_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd102c2efc0ecfb1e3d8d0927cd612413';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    # 相应的hand_pose_req msg中header的seq,表示是对该帧pose的相应
    uint32 resp_frame_id     
    # 0-失败， 1-成功， 2-默认值(default），其他数值暂不可用，留待后续扩展   
    uint8 hand_move_success      
    uint8 head_move_success     
    uint8 gripper_move_success 
    uint8 waist_move_success 
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pose_action_status(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.resp_frame_id !== undefined) {
      resolved.resp_frame_id = msg.resp_frame_id;
    }
    else {
      resolved.resp_frame_id = 0
    }

    if (msg.hand_move_success !== undefined) {
      resolved.hand_move_success = msg.hand_move_success;
    }
    else {
      resolved.hand_move_success = 0
    }

    if (msg.head_move_success !== undefined) {
      resolved.head_move_success = msg.head_move_success;
    }
    else {
      resolved.head_move_success = 0
    }

    if (msg.gripper_move_success !== undefined) {
      resolved.gripper_move_success = msg.gripper_move_success;
    }
    else {
      resolved.gripper_move_success = 0
    }

    if (msg.waist_move_success !== undefined) {
      resolved.waist_move_success = msg.waist_move_success;
    }
    else {
      resolved.waist_move_success = 0
    }

    return resolved;
    }
};

module.exports = pose_action_status;
