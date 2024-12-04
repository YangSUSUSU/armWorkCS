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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class hand_pose_req {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.hand_move_enable = null;
      this.hand_side = null;
      this.hand_reset = null;
      this.pose_req = null;
      this.head_move_enable = null;
      this.rpy = null;
      this.gripper_move_enable = null;
      this.gripper_side = null;
      this.finger_status = null;
      this.little_finger = null;
      this.ring_finger = null;
      this.middle_finger = null;
      this.index_finger = null;
      this.thumb_bending = null;
      this.thumb_rotating = null;
      this.waist_enable = null;
      this.waist_rotate_ang = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('hand_move_enable')) {
        this.hand_move_enable = initObj.hand_move_enable
      }
      else {
        this.hand_move_enable = 0;
      }
      if (initObj.hasOwnProperty('hand_side')) {
        this.hand_side = initObj.hand_side
      }
      else {
        this.hand_side = 0;
      }
      if (initObj.hasOwnProperty('hand_reset')) {
        this.hand_reset = initObj.hand_reset
      }
      else {
        this.hand_reset = 0;
      }
      if (initObj.hasOwnProperty('pose_req')) {
        this.pose_req = initObj.pose_req
      }
      else {
        this.pose_req = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('head_move_enable')) {
        this.head_move_enable = initObj.head_move_enable
      }
      else {
        this.head_move_enable = 0;
      }
      if (initObj.hasOwnProperty('rpy')) {
        this.rpy = initObj.rpy
      }
      else {
        this.rpy = [];
      }
      if (initObj.hasOwnProperty('gripper_move_enable')) {
        this.gripper_move_enable = initObj.gripper_move_enable
      }
      else {
        this.gripper_move_enable = 0;
      }
      if (initObj.hasOwnProperty('gripper_side')) {
        this.gripper_side = initObj.gripper_side
      }
      else {
        this.gripper_side = 0;
      }
      if (initObj.hasOwnProperty('finger_status')) {
        this.finger_status = initObj.finger_status
      }
      else {
        this.finger_status = 0;
      }
      if (initObj.hasOwnProperty('little_finger')) {
        this.little_finger = initObj.little_finger
      }
      else {
        this.little_finger = 0.0;
      }
      if (initObj.hasOwnProperty('ring_finger')) {
        this.ring_finger = initObj.ring_finger
      }
      else {
        this.ring_finger = 0.0;
      }
      if (initObj.hasOwnProperty('middle_finger')) {
        this.middle_finger = initObj.middle_finger
      }
      else {
        this.middle_finger = 0.0;
      }
      if (initObj.hasOwnProperty('index_finger')) {
        this.index_finger = initObj.index_finger
      }
      else {
        this.index_finger = 0.0;
      }
      if (initObj.hasOwnProperty('thumb_bending')) {
        this.thumb_bending = initObj.thumb_bending
      }
      else {
        this.thumb_bending = 0.0;
      }
      if (initObj.hasOwnProperty('thumb_rotating')) {
        this.thumb_rotating = initObj.thumb_rotating
      }
      else {
        this.thumb_rotating = 0.0;
      }
      if (initObj.hasOwnProperty('waist_enable')) {
        this.waist_enable = initObj.waist_enable
      }
      else {
        this.waist_enable = 0;
      }
      if (initObj.hasOwnProperty('waist_rotate_ang')) {
        this.waist_rotate_ang = initObj.waist_rotate_ang
      }
      else {
        this.waist_rotate_ang = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hand_pose_req
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [hand_move_enable]
    bufferOffset = _serializer.uint8(obj.hand_move_enable, buffer, bufferOffset);
    // Serialize message field [hand_side]
    bufferOffset = _serializer.uint8(obj.hand_side, buffer, bufferOffset);
    // Serialize message field [hand_reset]
    bufferOffset = _serializer.uint8(obj.hand_reset, buffer, bufferOffset);
    // Serialize message field [pose_req]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose_req, buffer, bufferOffset);
    // Serialize message field [head_move_enable]
    bufferOffset = _serializer.uint8(obj.head_move_enable, buffer, bufferOffset);
    // Serialize message field [rpy]
    bufferOffset = _arraySerializer.float64(obj.rpy, buffer, bufferOffset, null);
    // Serialize message field [gripper_move_enable]
    bufferOffset = _serializer.uint8(obj.gripper_move_enable, buffer, bufferOffset);
    // Serialize message field [gripper_side]
    bufferOffset = _serializer.uint8(obj.gripper_side, buffer, bufferOffset);
    // Serialize message field [finger_status]
    bufferOffset = _serializer.uint8(obj.finger_status, buffer, bufferOffset);
    // Serialize message field [little_finger]
    bufferOffset = _serializer.float32(obj.little_finger, buffer, bufferOffset);
    // Serialize message field [ring_finger]
    bufferOffset = _serializer.float32(obj.ring_finger, buffer, bufferOffset);
    // Serialize message field [middle_finger]
    bufferOffset = _serializer.float32(obj.middle_finger, buffer, bufferOffset);
    // Serialize message field [index_finger]
    bufferOffset = _serializer.float32(obj.index_finger, buffer, bufferOffset);
    // Serialize message field [thumb_bending]
    bufferOffset = _serializer.float32(obj.thumb_bending, buffer, bufferOffset);
    // Serialize message field [thumb_rotating]
    bufferOffset = _serializer.float32(obj.thumb_rotating, buffer, bufferOffset);
    // Serialize message field [waist_enable]
    bufferOffset = _serializer.uint8(obj.waist_enable, buffer, bufferOffset);
    // Serialize message field [waist_rotate_ang]
    bufferOffset = _serializer.float32(obj.waist_rotate_ang, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hand_pose_req
    let len;
    let data = new hand_pose_req(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [hand_move_enable]
    data.hand_move_enable = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hand_side]
    data.hand_side = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hand_reset]
    data.hand_reset = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pose_req]
    data.pose_req = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [head_move_enable]
    data.head_move_enable = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rpy]
    data.rpy = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [gripper_move_enable]
    data.gripper_move_enable = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gripper_side]
    data.gripper_side = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [finger_status]
    data.finger_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [little_finger]
    data.little_finger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ring_finger]
    data.ring_finger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [middle_finger]
    data.middle_finger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [index_finger]
    data.index_finger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thumb_bending]
    data.thumb_bending = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thumb_rotating]
    data.thumb_rotating = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [waist_enable]
    data.waist_enable = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [waist_rotate_ang]
    data.waist_rotate_ang = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.rpy.length;
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'llm_msgs/hand_pose_req';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6b3b996b8f56c6f6190cb1b04d5143a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header     
    
    # 以下分别为手臂、头部、手指对应的控制数据，相应数据仅当对应使能位有效时才有意义
    # 例如: 当hand_move_enable为1(enable状态)时, pose_req的数据才需要被执行，才有意义
    
    # 手臂移动使能位-hand_move_enable, 手臂目标位姿-pose_req, 手臂复位-hand_reset(0-非复位位姿，1-复位位姿)
    uint8 hand_move_enable    
    uint8 hand_side 
    uint8 hand_reset          
    geometry_msgs/Pose pose_req 
    
    # 头部移动使能位-move_enable, 头部目标位姿-(R, P, Y）
    uint8 head_move_enable    
    float64[] rpy
    
    # 手部手指移动使能位-gripper_move_enable, 手指开度-[figure_1, figure_2, figure_3, figure_4, figure_5]
    uint8 gripper_move_enable 
    uint8 gripper_side  
    uint8 finger_status
    float32 little_finger
    float32 ring_finger
    float32 middle_finger
    float32 index_finger
    float32 thumb_bending
    float32 thumb_rotating
    
    # 腰部转动
    uint8 waist_enable # 0-disable, 1-enable, 2-default, 其他数值暂不可用，留待后续
    float32 waist_rotate_ang # 腰部旋转角度-rad
    
    
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
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hand_pose_req(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.hand_move_enable !== undefined) {
      resolved.hand_move_enable = msg.hand_move_enable;
    }
    else {
      resolved.hand_move_enable = 0
    }

    if (msg.hand_side !== undefined) {
      resolved.hand_side = msg.hand_side;
    }
    else {
      resolved.hand_side = 0
    }

    if (msg.hand_reset !== undefined) {
      resolved.hand_reset = msg.hand_reset;
    }
    else {
      resolved.hand_reset = 0
    }

    if (msg.pose_req !== undefined) {
      resolved.pose_req = geometry_msgs.msg.Pose.Resolve(msg.pose_req)
    }
    else {
      resolved.pose_req = new geometry_msgs.msg.Pose()
    }

    if (msg.head_move_enable !== undefined) {
      resolved.head_move_enable = msg.head_move_enable;
    }
    else {
      resolved.head_move_enable = 0
    }

    if (msg.rpy !== undefined) {
      resolved.rpy = msg.rpy;
    }
    else {
      resolved.rpy = []
    }

    if (msg.gripper_move_enable !== undefined) {
      resolved.gripper_move_enable = msg.gripper_move_enable;
    }
    else {
      resolved.gripper_move_enable = 0
    }

    if (msg.gripper_side !== undefined) {
      resolved.gripper_side = msg.gripper_side;
    }
    else {
      resolved.gripper_side = 0
    }

    if (msg.finger_status !== undefined) {
      resolved.finger_status = msg.finger_status;
    }
    else {
      resolved.finger_status = 0
    }

    if (msg.little_finger !== undefined) {
      resolved.little_finger = msg.little_finger;
    }
    else {
      resolved.little_finger = 0.0
    }

    if (msg.ring_finger !== undefined) {
      resolved.ring_finger = msg.ring_finger;
    }
    else {
      resolved.ring_finger = 0.0
    }

    if (msg.middle_finger !== undefined) {
      resolved.middle_finger = msg.middle_finger;
    }
    else {
      resolved.middle_finger = 0.0
    }

    if (msg.index_finger !== undefined) {
      resolved.index_finger = msg.index_finger;
    }
    else {
      resolved.index_finger = 0.0
    }

    if (msg.thumb_bending !== undefined) {
      resolved.thumb_bending = msg.thumb_bending;
    }
    else {
      resolved.thumb_bending = 0.0
    }

    if (msg.thumb_rotating !== undefined) {
      resolved.thumb_rotating = msg.thumb_rotating;
    }
    else {
      resolved.thumb_rotating = 0.0
    }

    if (msg.waist_enable !== undefined) {
      resolved.waist_enable = msg.waist_enable;
    }
    else {
      resolved.waist_enable = 0
    }

    if (msg.waist_rotate_ang !== undefined) {
      resolved.waist_rotate_ang = msg.waist_rotate_ang;
    }
    else {
      resolved.waist_rotate_ang = 0.0
    }

    return resolved;
    }
};

module.exports = hand_pose_req;
