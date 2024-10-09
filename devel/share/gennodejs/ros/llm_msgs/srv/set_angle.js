// Auto-generated. Do not edit!

// (in-package llm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class set_angleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle0Ratio = null;
      this.angle1Ratio = null;
      this.angle2Ratio = null;
      this.angle3Ratio = null;
      this.angle4Ratio = null;
      this.angle5Ratio = null;
    }
    else {
      if (initObj.hasOwnProperty('angle0Ratio')) {
        this.angle0Ratio = initObj.angle0Ratio
      }
      else {
        this.angle0Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('angle1Ratio')) {
        this.angle1Ratio = initObj.angle1Ratio
      }
      else {
        this.angle1Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('angle2Ratio')) {
        this.angle2Ratio = initObj.angle2Ratio
      }
      else {
        this.angle2Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('angle3Ratio')) {
        this.angle3Ratio = initObj.angle3Ratio
      }
      else {
        this.angle3Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('angle4Ratio')) {
        this.angle4Ratio = initObj.angle4Ratio
      }
      else {
        this.angle4Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('angle5Ratio')) {
        this.angle5Ratio = initObj.angle5Ratio
      }
      else {
        this.angle5Ratio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_angleRequest
    // Serialize message field [angle0Ratio]
    bufferOffset = _serializer.float32(obj.angle0Ratio, buffer, bufferOffset);
    // Serialize message field [angle1Ratio]
    bufferOffset = _serializer.float32(obj.angle1Ratio, buffer, bufferOffset);
    // Serialize message field [angle2Ratio]
    bufferOffset = _serializer.float32(obj.angle2Ratio, buffer, bufferOffset);
    // Serialize message field [angle3Ratio]
    bufferOffset = _serializer.float32(obj.angle3Ratio, buffer, bufferOffset);
    // Serialize message field [angle4Ratio]
    bufferOffset = _serializer.float32(obj.angle4Ratio, buffer, bufferOffset);
    // Serialize message field [angle5Ratio]
    bufferOffset = _serializer.float32(obj.angle5Ratio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_angleRequest
    let len;
    let data = new set_angleRequest(null);
    // Deserialize message field [angle0Ratio]
    data.angle0Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle1Ratio]
    data.angle1Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle2Ratio]
    data.angle2Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle3Ratio]
    data.angle3Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle4Ratio]
    data.angle4Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle5Ratio]
    data.angle5Ratio = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_angleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc73e456d4729e71af66373d93d820b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 angle0Ratio
    float32 angle1Ratio
    float32 angle2Ratio
    float32 angle3Ratio
    float32 angle4Ratio
    float32 angle5Ratio
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_angleRequest(null);
    if (msg.angle0Ratio !== undefined) {
      resolved.angle0Ratio = msg.angle0Ratio;
    }
    else {
      resolved.angle0Ratio = 0.0
    }

    if (msg.angle1Ratio !== undefined) {
      resolved.angle1Ratio = msg.angle1Ratio;
    }
    else {
      resolved.angle1Ratio = 0.0
    }

    if (msg.angle2Ratio !== undefined) {
      resolved.angle2Ratio = msg.angle2Ratio;
    }
    else {
      resolved.angle2Ratio = 0.0
    }

    if (msg.angle3Ratio !== undefined) {
      resolved.angle3Ratio = msg.angle3Ratio;
    }
    else {
      resolved.angle3Ratio = 0.0
    }

    if (msg.angle4Ratio !== undefined) {
      resolved.angle4Ratio = msg.angle4Ratio;
    }
    else {
      resolved.angle4Ratio = 0.0
    }

    if (msg.angle5Ratio !== undefined) {
      resolved.angle5Ratio = msg.angle5Ratio;
    }
    else {
      resolved.angle5Ratio = 0.0
    }

    return resolved;
    }
};

class set_angleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_accepted = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_accepted')) {
        this.angle_accepted = initObj.angle_accepted
      }
      else {
        this.angle_accepted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_angleResponse
    // Serialize message field [angle_accepted]
    bufferOffset = _serializer.bool(obj.angle_accepted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_angleResponse
    let len;
    let data = new set_angleResponse(null);
    // Deserialize message field [angle_accepted]
    data.angle_accepted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_angleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1508b076c4c46f43d5103fcfc81271e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool angle_accepted
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_angleResponse(null);
    if (msg.angle_accepted !== undefined) {
      resolved.angle_accepted = msg.angle_accepted;
    }
    else {
      resolved.angle_accepted = false
    }

    return resolved;
    }
};

module.exports = {
  Request: set_angleRequest,
  Response: set_angleResponse,
  md5sum() { return '3f54cd874965b27feb654abae7abbdcb'; },
  datatype() { return 'llm_msgs/set_angle'; }
};
