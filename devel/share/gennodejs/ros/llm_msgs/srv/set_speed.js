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

class set_speedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed0Ratio = null;
      this.speed1Ratio = null;
      this.speed2Ratio = null;
      this.speed3Ratio = null;
      this.speed4Ratio = null;
      this.speed5Ratio = null;
    }
    else {
      if (initObj.hasOwnProperty('speed0Ratio')) {
        this.speed0Ratio = initObj.speed0Ratio
      }
      else {
        this.speed0Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('speed1Ratio')) {
        this.speed1Ratio = initObj.speed1Ratio
      }
      else {
        this.speed1Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('speed2Ratio')) {
        this.speed2Ratio = initObj.speed2Ratio
      }
      else {
        this.speed2Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('speed3Ratio')) {
        this.speed3Ratio = initObj.speed3Ratio
      }
      else {
        this.speed3Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('speed4Ratio')) {
        this.speed4Ratio = initObj.speed4Ratio
      }
      else {
        this.speed4Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('speed5Ratio')) {
        this.speed5Ratio = initObj.speed5Ratio
      }
      else {
        this.speed5Ratio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_speedRequest
    // Serialize message field [speed0Ratio]
    bufferOffset = _serializer.float32(obj.speed0Ratio, buffer, bufferOffset);
    // Serialize message field [speed1Ratio]
    bufferOffset = _serializer.float32(obj.speed1Ratio, buffer, bufferOffset);
    // Serialize message field [speed2Ratio]
    bufferOffset = _serializer.float32(obj.speed2Ratio, buffer, bufferOffset);
    // Serialize message field [speed3Ratio]
    bufferOffset = _serializer.float32(obj.speed3Ratio, buffer, bufferOffset);
    // Serialize message field [speed4Ratio]
    bufferOffset = _serializer.float32(obj.speed4Ratio, buffer, bufferOffset);
    // Serialize message field [speed5Ratio]
    bufferOffset = _serializer.float32(obj.speed5Ratio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_speedRequest
    let len;
    let data = new set_speedRequest(null);
    // Deserialize message field [speed0Ratio]
    data.speed0Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed1Ratio]
    data.speed1Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed2Ratio]
    data.speed2Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed3Ratio]
    data.speed3Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed4Ratio]
    data.speed4Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed5Ratio]
    data.speed5Ratio = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_speedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fce368f208823771f7ce8907032a98d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 speed0Ratio
    float32 speed1Ratio
    float32 speed2Ratio
    float32 speed3Ratio
    float32 speed4Ratio
    float32 speed5Ratio
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_speedRequest(null);
    if (msg.speed0Ratio !== undefined) {
      resolved.speed0Ratio = msg.speed0Ratio;
    }
    else {
      resolved.speed0Ratio = 0.0
    }

    if (msg.speed1Ratio !== undefined) {
      resolved.speed1Ratio = msg.speed1Ratio;
    }
    else {
      resolved.speed1Ratio = 0.0
    }

    if (msg.speed2Ratio !== undefined) {
      resolved.speed2Ratio = msg.speed2Ratio;
    }
    else {
      resolved.speed2Ratio = 0.0
    }

    if (msg.speed3Ratio !== undefined) {
      resolved.speed3Ratio = msg.speed3Ratio;
    }
    else {
      resolved.speed3Ratio = 0.0
    }

    if (msg.speed4Ratio !== undefined) {
      resolved.speed4Ratio = msg.speed4Ratio;
    }
    else {
      resolved.speed4Ratio = 0.0
    }

    if (msg.speed5Ratio !== undefined) {
      resolved.speed5Ratio = msg.speed5Ratio;
    }
    else {
      resolved.speed5Ratio = 0.0
    }

    return resolved;
    }
};

class set_speedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed_accepted = null;
    }
    else {
      if (initObj.hasOwnProperty('speed_accepted')) {
        this.speed_accepted = initObj.speed_accepted
      }
      else {
        this.speed_accepted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_speedResponse
    // Serialize message field [speed_accepted]
    bufferOffset = _serializer.bool(obj.speed_accepted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_speedResponse
    let len;
    let data = new set_speedResponse(null);
    // Deserialize message field [speed_accepted]
    data.speed_accepted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_speedResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bb3ff7624b31e52edd1d0b2bbbae418';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool speed_accepted
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_speedResponse(null);
    if (msg.speed_accepted !== undefined) {
      resolved.speed_accepted = msg.speed_accepted;
    }
    else {
      resolved.speed_accepted = false
    }

    return resolved;
    }
};

module.exports = {
  Request: set_speedRequest,
  Response: set_speedResponse,
  md5sum() { return '4d1d00d67ce0ba0765e4ad67b563d391'; },
  datatype() { return 'llm_msgs/set_speed'; }
};
