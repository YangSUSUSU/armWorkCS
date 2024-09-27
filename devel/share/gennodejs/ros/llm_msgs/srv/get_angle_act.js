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

class get_angle_actRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_angle_actRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_angle_actRequest
    let len;
    let data = new get_angle_actRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/get_angle_actRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_angle_actRequest(null);
    return resolved;
    }
};

class get_angle_actResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.curangleRatio = null;
    }
    else {
      if (initObj.hasOwnProperty('curangleRatio')) {
        this.curangleRatio = initObj.curangleRatio
      }
      else {
        this.curangleRatio = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_angle_actResponse
    // Check that the constant length array field [curangleRatio] has the right length
    if (obj.curangleRatio.length !== 6) {
      throw new Error('Unable to serialize array field curangleRatio - length must be 6')
    }
    // Serialize message field [curangleRatio]
    bufferOffset = _arraySerializer.float32(obj.curangleRatio, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_angle_actResponse
    let len;
    let data = new get_angle_actResponse(null);
    // Deserialize message field [curangleRatio]
    data.curangleRatio = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/get_angle_actResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a40d1cc41b6533f906a6a90344676b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[6] curangleRatio
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_angle_actResponse(null);
    if (msg.curangleRatio !== undefined) {
      resolved.curangleRatio = msg.curangleRatio;
    }
    else {
      resolved.curangleRatio = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: get_angle_actRequest,
  Response: get_angle_actResponse,
  md5sum() { return '5a40d1cc41b6533f906a6a90344676b6'; },
  datatype() { return 'llm_msgs/get_angle_act'; }
};
