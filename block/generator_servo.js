module.exports = function (Blockly) {

  // =============================================================================
  // I/O
  // =============================================================================

  Blockly.JavaScript['esp32_servo_attach'] = function (block) {
    var text_pin = block.getFieldValue('pin');
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var code = `#EXTINC#include "Servo.h"#END
#VARIABLE Servo ${variable_instance};#END
${variable_instance}.attach(${text_pin});
`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_detach'] = function (block) {
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var code = `${variable_instance}.detach;\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_write'] = function (block) {
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var value_degree = Blockly.JavaScript.valueToCode(block, 'degree', Blockly.JavaScript.ORDER_ATOMIC);
    var code = `${variable_instance}.write(${value_degree});\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_write_micros'] = function (block) {
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var value_degree = Blockly.JavaScript.valueToCode(block, 'degree', Blockly.JavaScript.ORDER_ATOMIC);
    var code = `${variable_instance}.writeMicroseconds(${value_degree});\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_read'] = function (block) {
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var code = `${variable_instance}.read()`;
    return [code, Blockly.JavaScript.ORDER_NONE];
  };

  Blockly.JavaScript['esp32_servo_read_micros'] = function (block) {
    var variable_instance = Blockly.JavaScript.variableDB_.getName(block.getFieldValue('instance'), Blockly.Variables.NAME_TYPE);
    var code = `${variable_instance}.readMicroseconds()`;
    return [code, Blockly.JavaScript.ORDER_NONE];
  };

  Blockly.JavaScript['esp32_servo_gripper_setup'] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code = `
    #EXTINC#include"Servo.h"#END

#VARIABLE 
Servo gripper1;
Servo gripper2;
int i;
int degree;
int speedarrest;
int speedrelease;
int speedup;
int speeddown;
#END

gripper1.attach(15);
gripper2.attach(17);
gripper1.write(0);
gripper2.write(0);

`;
    return code;
  };

  //   Blockly.JavaScript['esp32_servo_gripper_Arrest'] = function (block) {
  //     var value_degree = Blockly.JavaScript.valueToCode(block, 'degree', Blockly.JavaScript.ORDER_ATOMIC);
  //     var number_speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ATOMIC);
  //     // TODO: Assemble JavaScript into code variable.
  //     var code = `
  // #VARIABLE 
  // int i;
  // int mappedVal;
  // #END
  //     for (i = 0; i <= ${value_degree}; i++) {
  //       gripper1.write(i);
  //       Serial.println(i);
  //       mappedVal=map(${number_speed}, 0, 100, 30, 0);
  //       delay(mappedVal);
  //     }\n`;
  //     return code;
  //   };

  Blockly.JavaScript['esp32_servo_gripper_arrest'] = function (block) {
    var value_degree = Blockly.JavaScript.valueToCode(block, 'degree', Blockly.JavaScript.ORDER_ATOMIC);
    var number_speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ATOMIC);
    // TODO: Assemble JavaScript into code variable.
    var code = `
degree =${value_degree};
if (degree > 70) {
  degree = 70;
}
    speedarrest = map(${number_speed}, 0, 100, 30, 0);
        for (i = 0; i <= degree; i++) {
          gripper1.write(i);
          delay(speedarrest);
        }\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_gripper_release'] = function (block) {
    var number_speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ATOMIC);
    // TODO: Assemble JavaScript into code variable.
    var code = `
speedrelease = map(${number_speed}, 0, 100, 30, 0);
for (i = 90; i >= 0; i--) {
      gripper1.write(i);
      delay(speedrelease);
    }\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_gripper_down'] = function (block) {
    var number_speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ATOMIC);
    // TODO: Assemble JavaScript into code variable.
    var code = `
speedup = map(${number_speed}, 0, 100, 30, 0);
    for (i = 0; i <= 90; i++) {
      gripper2.write(i);
      delay(speedup);
    }\n`;
    return code;
  };

  Blockly.JavaScript['esp32_servo_gripper_up'] = function (block) {
    var number_speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ATOMIC);
    // TODO: Assemble JavaScript into code variable.
    var code = `
speeddown = map(${number_speed}, 0, 100, 30, 0);
for (i = 90; i >= 0; i--) {
      gripper2.write(i);
      delay(speeddown);
    }\n`;
    return code;
  };

}