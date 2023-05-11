module.exports = function (Blockly) {
  'use strict';

  // =============================================================================
  // Servo
  // =============================================================================

  Blockly.Blocks['esp32_servo_attach'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("attach pin")

        .appendField(new Blockly.FieldTextInput("15"), "pin")
      // .appendField(new Blockly.FieldDropdown([
      //   ["1", "15"],
      //   ["2", "17"],

      // ]), 'pin')

      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(75);
      this.setTooltip("Associate this instance with a servomotor whose input is connected to pin.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_detach'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("detach");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(75);
      this.setTooltip("Stop driving the servo pulse train.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_write'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("write angle");
      this.appendValueInput("degree")
        .setCheck("Number");
      this.appendDummyInput()
        .appendField(" degree");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(75);
      this.setTooltip("Set the servomotor target angle.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_write_micros'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("write pulse");
      this.appendValueInput("degree")
        .setCheck("Number");
      this.appendDummyInput()
        .appendField("microseconds");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(75);
      this.setTooltip("Set the pulse width, in microseconds.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_read'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("read angle");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(75);
      this.setTooltip("Get the servomotor's target angle, in degrees.  This will lie inside the range specified at attach() time.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_read_micros'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldVariable("Servo1", null, ["Plugin.Servo"], ["Plugin.Servo"]), "instance")
        .appendField("read pulse microsec");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(75);
      this.setTooltip("Get the current pulse width, in microseconds.  This will lie within the range specified at attach() time.");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_gripper_setup'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Gripper : Setup")
      //.appendField(new Blockly.FieldTextInput("0x42"), "ADDS");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(320);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_gripper_arrest'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Gripper : Arrest");
      this.appendValueInput("degree")
        .setCheck("Number")
      this.appendDummyInput()
        .appendField("degree [ 0-70 ]")
      this.appendDummyInput()
        .appendField(": Speed");
      this.appendValueInput("SPEED")
        .setCheck("Number")
      this.appendDummyInput()
        .appendField("0-100%")
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(320);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_gripper_release'] = {
    init: function () {
      this.appendDummyInput()
      .appendField("Gripper : Release");
      this.appendDummyInput()
        .appendField(": Speed");
      this.appendValueInput("SPEED")
        .setCheck("Number")
      this.appendDummyInput()
        .appendField("0-100%")
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(320);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_gripper_up'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Gripper : Hand Up ");
      this.appendDummyInput()
        .appendField(": Speed");
      this.appendValueInput("SPEED")
        .setCheck("Number")
      this.appendDummyInput()
        .appendField("0-100%")
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(320);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['esp32_servo_gripper_down'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Gripper : Hand Down ");
        this.appendDummyInput()
        .appendField(": Speed");
      this.appendValueInput("SPEED")
        .setCheck("Number")
      this.appendDummyInput()
        .appendField("0-100%")
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(320);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

}